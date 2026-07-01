#!/usr/bin/env python3
"""Benchmark how many Sionna RT dataset workers one GPU can run concurrently.

This script intentionally writes every worker into an independent temporary
dataset root under Server_performance_eval/runs/. It does not touch the main
RT_dataset/database files, so concurrent benchmark workers cannot corrupt the
real dataset metadata.

Typical quick test:
    conda activate sionna_rt
    python RT_dataset/Server_performance_eval/benchmark_4090_task_limit.py \
        --gpu-id 0 --max-workers 6

Full-workload test matching the current RT_dataset config:
    python RT_dataset/Server_performance_eval/benchmark_4090_task_limit.py \
        --gpu-id 0 --max-workers 4 --full-workload
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable


SCRIPT_PATH = Path(__file__).resolve()
EVAL_ROOT = SCRIPT_PATH.parent
RT_DATASET_ROOT = EVAL_ROOT.parent
PROJECT_ROOT = RT_DATASET_ROOT.parent
DEFAULT_GENERATOR = RT_DATASET_ROOT / "scripts" / "build_rt_uav_stft_dataset.py"
DEFAULT_CONFIG = RT_DATASET_ROOT / "configs" / "rt_dataset_config.yaml"

OOM_PATTERNS = [
    re.compile(r"out of memory", re.IGNORECASE),
    re.compile(r"CUDA_ERROR_OUT_OF_MEMORY", re.IGNORECASE),
    re.compile(r"RESOURCE_EXHAUSTED", re.IGNORECASE),
    re.compile(r"failed to allocate", re.IGNORECASE),
    re.compile(r"OOM", re.IGNORECASE),
]


@dataclass
class WorkerResult:
    worker_id: int
    returncode: int | None
    elapsed_s: float
    log_path: Path
    root_path: Path
    samples_written: int
    oom_detected: bool


@dataclass
class StageResult:
    concurrency: int
    status: str
    elapsed_s: float
    peak_memory_used_mb: int | None
    peak_gpu_util_percent: int | None
    samples_written: int
    worker_results: list[WorkerResult]
    stage_dir: Path
    failure_reason: str


def run_cmd(cmd: list[str], timeout_s: float = 15.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout_s,
        check=False,
    )


def nvidia_smi_query(gpu_id: int) -> dict[str, str] | None:
    cmd = [
        "nvidia-smi",
        "--id",
        str(gpu_id),
        "--query-gpu=index,name,memory.used,memory.total,utilization.gpu,power.draw,temperature.gpu",
        "--format=csv,noheader,nounits",
    ]
    try:
        proc = run_cmd(cmd)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0 or not proc.stdout.strip():
        return None
    row = next(csv.reader([proc.stdout.strip().splitlines()[0]]))
    keys = [
        "index",
        "name",
        "memory_used_mb",
        "memory_total_mb",
        "gpu_util_percent",
        "power_w",
        "temperature_c",
    ]
    return {key: value.strip() for key, value in zip(keys, row)}


def parse_int(value: str | None) -> int | None:
    if value is None:
        return None
    match = re.search(r"-?\d+", str(value))
    return int(match.group(0)) if match else None


def monitor_gpu(
    *,
    gpu_id: int,
    interval_s: float,
    stop_event: threading.Event,
    output_csv: Path,
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "unix_time",
        "local_time",
        "gpu_id",
        "name",
        "memory_used_mb",
        "memory_total_mb",
        "gpu_util_percent",
        "power_w",
        "temperature_c",
    ]
    with output_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        while not stop_event.is_set():
            snap = nvidia_smi_query(gpu_id)
            now = time.time()
            if snap is not None:
                row = {
                    "unix_time": f"{now:.3f}",
                    "local_time": datetime.now().isoformat(timespec="seconds"),
                    "gpu_id": gpu_id,
                    "name": snap.get("name", ""),
                    "memory_used_mb": snap.get("memory_used_mb", ""),
                    "memory_total_mb": snap.get("memory_total_mb", ""),
                    "gpu_util_percent": snap.get("gpu_util_percent", ""),
                    "power_w": snap.get("power_w", ""),
                    "temperature_c": snap.get("temperature_c", ""),
                }
                writer.writerow(row)
                f.flush()
            stop_event.wait(interval_s)


def summarize_monitor_csv(path: Path) -> tuple[int | None, int | None]:
    if not path.exists():
        return None, None
    peak_mem: int | None = None
    peak_util: int | None = None
    with path.open("r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            mem = parse_int(row.get("memory_used_mb"))
            util = parse_int(row.get("gpu_util_percent"))
            if mem is not None:
                peak_mem = mem if peak_mem is None else max(peak_mem, mem)
            if util is not None:
                peak_util = util if peak_util is None else max(peak_util, util)
    return peak_mem, peak_util


def count_timing_rows(root: Path) -> int:
    timing_path = root / "database" / "timing.csv"
    if not timing_path.exists():
        return 0
    with timing_path.open("r", encoding="utf-8", newline="") as f:
        return sum(1 for _ in csv.DictReader(f))


def log_has_oom(path: Path) -> bool:
    if not path.exists():
        return False
    text = path.read_text(encoding="utf-8", errors="ignore")
    return any(pattern.search(text) for pattern in OOM_PATTERNS)


def tail_text(path: Path, max_lines: int = 30) -> str:
    if not path.exists():
        return ""
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    return "\n".join(lines[-max_lines:])


def build_worker_command(
    *,
    python_bin: str,
    generator: Path,
    worker_root: Path,
    config: Path,
    samples_per_worker: int,
    classes: list[str],
    full_workload: bool,
    snapshot_override: int,
    rt_snapshot_stride: int,
) -> list[str]:
    cmd = [
        python_bin,
        str(generator),
        "--root",
        str(worker_root),
        "--config",
        str(config),
        "--samples-per-class",
        str(samples_per_worker),
        "--classes",
        *classes,
    ]
    if not full_workload:
        cmd.extend(["--snapshot-override", str(snapshot_override)])
        cmd.extend(["--rt-snapshot-stride", str(rt_snapshot_stride)])
    return cmd


def terminate_process(proc: subprocess.Popen[str], grace_s: float = 20.0) -> None:
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=10)


def run_stage(args: argparse.Namespace, run_dir: Path, concurrency: int) -> StageResult:
    stage_dir = run_dir / f"concurrency_{concurrency:02d}"
    stage_dir.mkdir(parents=True, exist_ok=True)
    monitor_csv = stage_dir / "gpu_monitor.csv"
    stop_event = threading.Event()
    monitor_thread = threading.Thread(
        target=monitor_gpu,
        kwargs={
            "gpu_id": args.gpu_id,
            "interval_s": args.monitor_interval_s,
            "stop_event": stop_event,
            "output_csv": monitor_csv,
        },
        daemon=True,
    )

    processes: list[tuple[int, subprocess.Popen[str], Path, Path, float]] = []
    stage_start = time.perf_counter()
    monitor_thread.start()
    try:
        for worker_id in range(concurrency):
            worker_root = stage_dir / f"worker_{worker_id:02d}_dataset"
            log_path = stage_dir / f"worker_{worker_id:02d}.log"
            cmd = build_worker_command(
                python_bin=args.python,
                generator=args.generator,
                worker_root=worker_root,
                config=args.config,
                samples_per_worker=args.samples_per_worker,
                classes=args.classes,
                full_workload=args.full_workload,
                snapshot_override=args.snapshot_override,
                rt_snapshot_stride=args.rt_snapshot_stride,
            )
            env = os.environ.copy()
            env["CUDA_VISIBLE_DEVICES"] = str(args.gpu_id)
            if args.enable_tf_memory_growth:
                env.setdefault("TF_FORCE_GPU_ALLOW_GROWTH", "true")
                env.setdefault("TF_GPU_ALLOCATOR", "cuda_malloc_async")
            with log_path.open("w", encoding="utf-8") as log:
                log.write("# Command\n")
                log.write(" ".join(cmd) + "\n\n")
                log.flush()
                proc = subprocess.Popen(
                    cmd,
                    cwd=str(PROJECT_ROOT),
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
            processes.append((worker_id, proc, log_path, worker_root, time.perf_counter()))
            if args.stagger_s > 0 and worker_id != concurrency - 1:
                time.sleep(args.stagger_s)

        timed_out = False
        while any(proc.poll() is None for _, proc, _, _, _ in processes):
            if args.timeout_s > 0 and time.perf_counter() - stage_start > args.timeout_s:
                timed_out = True
                for _, proc, _, _, _ in processes:
                    terminate_process(proc)
                break
            time.sleep(1.0)
    finally:
        stop_event.set()
        monitor_thread.join(timeout=max(2.0, args.monitor_interval_s + 1.0))

    stage_elapsed = time.perf_counter() - stage_start
    worker_results: list[WorkerResult] = []
    for worker_id, proc, log_path, worker_root, worker_start in processes:
        returncode = proc.poll()
        elapsed_s = max(0.0, time.perf_counter() - worker_start)
        worker_results.append(
            WorkerResult(
                worker_id=worker_id,
                returncode=returncode,
                elapsed_s=elapsed_s,
                log_path=log_path,
                root_path=worker_root,
                samples_written=count_timing_rows(worker_root),
                oom_detected=log_has_oom(log_path),
            )
        )

    peak_mem, peak_util = summarize_monitor_csv(monitor_csv)
    samples_written = sum(item.samples_written for item in worker_results)
    failed_workers = [item for item in worker_results if item.returncode != 0]
    oom_workers = [item for item in worker_results if item.oom_detected]
    expected_samples = concurrency * args.samples_per_worker * len(args.classes)

    failure_reason = ""
    status = "PASS"
    if timed_out:
        status = "FAIL"
        failure_reason = f"stage timeout after {args.timeout_s:.1f} s"
    elif failed_workers:
        status = "FAIL"
        failure_reason = "non-zero return code: " + ", ".join(
            f"worker {item.worker_id} rc={item.returncode}" for item in failed_workers
        )
    elif oom_workers:
        status = "FAIL"
        failure_reason = "OOM-like message detected in: " + ", ".join(
            f"worker {item.worker_id}" for item in oom_workers
        )
    elif samples_written < expected_samples:
        status = "FAIL"
        failure_reason = f"only {samples_written}/{expected_samples} expected samples were written"

    write_stage_summary(stage_dir, concurrency, status, failure_reason, worker_results)
    return StageResult(
        concurrency=concurrency,
        status=status,
        elapsed_s=stage_elapsed,
        peak_memory_used_mb=peak_mem,
        peak_gpu_util_percent=peak_util,
        samples_written=samples_written,
        worker_results=worker_results,
        stage_dir=stage_dir,
        failure_reason=failure_reason,
    )


def write_stage_summary(
    stage_dir: Path,
    concurrency: int,
    status: str,
    failure_reason: str,
    worker_results: Iterable[WorkerResult],
) -> None:
    path = stage_dir / "stage_summary.csv"
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "concurrency",
                "status",
                "failure_reason",
                "worker_id",
                "returncode",
                "elapsed_s",
                "samples_written",
                "oom_detected",
                "log_path",
                "root_path",
            ],
        )
        writer.writeheader()
        for item in worker_results:
            writer.writerow(
                {
                    "concurrency": concurrency,
                    "status": status,
                    "failure_reason": failure_reason,
                    "worker_id": item.worker_id,
                    "returncode": item.returncode,
                    "elapsed_s": f"{item.elapsed_s:.3f}",
                    "samples_written": item.samples_written,
                    "oom_detected": item.oom_detected,
                    "log_path": item.log_path,
                    "root_path": item.root_path,
                }
            )


def write_final_summary(run_dir: Path, args: argparse.Namespace, results: list[StageResult]) -> None:
    csv_path = run_dir / "summary.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "concurrency",
                "status",
                "elapsed_s",
                "samples_written",
                "samples_per_second",
                "peak_memory_used_mb",
                "peak_gpu_util_percent",
                "failure_reason",
                "stage_dir",
            ],
        )
        writer.writeheader()
        for item in results:
            samples_per_second = item.samples_written / item.elapsed_s if item.elapsed_s > 0 else 0.0
            writer.writerow(
                {
                    "concurrency": item.concurrency,
                    "status": item.status,
                    "elapsed_s": f"{item.elapsed_s:.3f}",
                    "samples_written": item.samples_written,
                    "samples_per_second": f"{samples_per_second:.6f}",
                    "peak_memory_used_mb": item.peak_memory_used_mb,
                    "peak_gpu_util_percent": item.peak_gpu_util_percent,
                    "failure_reason": item.failure_reason,
                    "stage_dir": item.stage_dir,
                }
            )

    passed = [item.concurrency for item in results if item.status == "PASS"]
    max_passed = max(passed) if passed else None
    first_failed = next((item.concurrency for item in results if item.status != "PASS"), None)
    gpu_info = nvidia_smi_query(args.gpu_id) or {}
    md_lines = [
        "# GPU Task Concurrency Benchmark",
        "",
        "## Configuration",
        "",
        f"- GPU id: `{args.gpu_id}`",
        f"- GPU name: `{gpu_info.get('name', 'unknown')}`",
        f"- Generator: `{args.generator}`",
        f"- Config: `{args.config}`",
        f"- Classes per worker: `{', '.join(args.classes)}`",
        f"- Samples per class per worker: `{args.samples_per_worker}`",
        f"- Full workload: `{args.full_workload}`",
        f"- Snapshot override when not full workload: `{args.snapshot_override}`",
        f"- RT snapshot stride when not full workload: `{args.rt_snapshot_stride}`",
        f"- TF memory growth env enabled: `{args.enable_tf_memory_growth}`",
        "",
        "## Result",
        "",
    ]
    if max_passed is None:
        md_lines.append("- No tested concurrency level passed.")
    elif first_failed is None:
        md_lines.append(
            f"- Tested up to `{max_passed}` concurrent workers and all passed. "
            f"The true upper bound is at least `{max_passed}` for this workload."
        )
    else:
        md_lines.append(
            f"- Largest passing concurrency: `{max_passed}` workers. "
            f"First failing concurrency: `{first_failed}` workers."
        )
    md_lines.extend(
        [
            "",
            "## Summary Table",
            "",
            "| workers | status | elapsed_s | samples | samples/s | peak_mem_MB | peak_util_% | failure |",
            "| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for item in results:
        samples_per_second = item.samples_written / item.elapsed_s if item.elapsed_s > 0 else 0.0
        failure = item.failure_reason.replace("|", "/")
        md_lines.append(
            f"| {item.concurrency} | {item.status} | {item.elapsed_s:.1f} | "
            f"{item.samples_written} | {samples_per_second:.4f} | "
            f"{item.peak_memory_used_mb if item.peak_memory_used_mb is not None else ''} | "
            f"{item.peak_gpu_util_percent if item.peak_gpu_util_percent is not None else ''} | "
            f"{failure} |"
        )
    md_lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "- This benchmark defines the limit as the largest worker count where every worker exits normally and writes all expected samples.",
            "- The result is workload-specific. `pitch30_v10` with 25 scatterers is much heavier than `single_blade_v0` with 1 scatterer.",
            "- For the most faithful production estimate, rerun with `--full-workload`; the quick default is designed to finish faster.",
            "- Each worker writes into its own shard directory, so this benchmark does not test shared CSV write contention.",
        ]
    )
    (run_dir / "report.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")


def update_latest_symlink(run_dir: Path) -> None:
    latest = EVAL_ROOT / "latest"
    try:
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(run_dir, target_is_directory=True)
    except OSError:
        pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Find a practical concurrent-worker limit for one GPU running the Sionna RT dataset generator."
    )
    parser.add_argument("--gpu-id", type=int, default=0, help="Physical GPU id shown by nvidia-smi.")
    parser.add_argument("--min-workers", type=int, default=1)
    parser.add_argument("--max-workers", type=int, default=8)
    parser.add_argument("--samples-per-worker", type=int, default=1)
    parser.add_argument("--classes", nargs="+", default=["pitch30_v10"], help="Dataset classes each worker generates.")
    parser.add_argument("--full-workload", action="store_true", help="Use config snapshots/stride exactly, usually slower.")
    parser.add_argument("--snapshot-override", type=int, default=256, help="Quick-test snapshot count.")
    parser.add_argument("--rt-snapshot-stride", type=int, default=4, help="Quick-test RT stride.")
    parser.add_argument("--timeout-s", type=float, default=0.0, help="Per concurrency stage timeout; 0 disables.")
    parser.add_argument("--monitor-interval-s", type=float, default=1.0)
    parser.add_argument("--stagger-s", type=float, default=0.0, help="Delay between launching workers.")
    parser.add_argument("--stop-on-failure", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--allow-busy-gpu", action="store_true", help="Run even if GPU already has notable memory use.")
    parser.add_argument("--busy-threshold-mb", type=int, default=1024)
    parser.add_argument("--enable-tf-memory-growth", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--generator", type=Path, default=DEFAULT_GENERATOR)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run-name", default="", help="Optional name under Server_performance_eval/runs/.")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.min_workers < 1:
        parser.error("--min-workers must be >= 1")
    if args.max_workers < args.min_workers:
        parser.error("--max-workers must be >= --min-workers")
    if args.samples_per_worker < 1:
        parser.error("--samples-per-worker must be >= 1")
    if args.snapshot_override < 1:
        parser.error("--snapshot-override must be >= 1")
    if args.rt_snapshot_stride < 1:
        parser.error("--rt-snapshot-stride must be >= 1")
    if not args.generator.exists():
        parser.error(f"generator not found: {args.generator}")
    if not args.config.exists():
        parser.error(f"config not found: {args.config}")
    return args


def main() -> int:
    args = parse_args()
    gpu = nvidia_smi_query(args.gpu_id)
    if gpu is None:
        print("ERROR: nvidia-smi is unavailable or the requested GPU id cannot be queried.", file=sys.stderr)
        return 2

    used_mb = parse_int(gpu.get("memory_used_mb")) or 0
    total_mb = parse_int(gpu.get("memory_total_mb")) or 0
    print(f"GPU {args.gpu_id}: {gpu.get('name')} memory {used_mb}/{total_mb} MiB")
    if used_mb > args.busy_threshold_mb and not args.allow_busy_gpu:
        print(
            f"ERROR: GPU already uses {used_mb} MiB, above threshold {args.busy_threshold_mb} MiB. "
            "Use --allow-busy-gpu if this is intentional.",
            file=sys.stderr,
        )
        return 3

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_name = args.run_name or f"gpu{args.gpu_id}_{timestamp}"
    run_dir = EVAL_ROOT / "runs" / run_name
    run_dir.mkdir(parents=True, exist_ok=False)
    update_latest_symlink(run_dir)

    command_preview = build_worker_command(
        python_bin=args.python,
        generator=args.generator,
        worker_root=run_dir / "concurrency_NN" / "worker_XX_dataset",
        config=args.config,
        samples_per_worker=args.samples_per_worker,
        classes=args.classes,
        full_workload=args.full_workload,
        snapshot_override=args.snapshot_override,
        rt_snapshot_stride=args.rt_snapshot_stride,
    )
    (run_dir / "benchmark_command.txt").write_text(
        " ".join(sys.argv) + "\n\nWorker command template:\n" + " ".join(command_preview) + "\n",
        encoding="utf-8",
    )

    if args.dry_run:
        print(f"Dry run only. Run directory prepared at: {run_dir}")
        print("Worker command template:")
        print(" ".join(command_preview))
        return 0

    results: list[StageResult] = []
    for concurrency in range(args.min_workers, args.max_workers + 1):
        print(f"\n=== Testing {concurrency} concurrent worker(s) ===", flush=True)
        result = run_stage(args, run_dir, concurrency)
        results.append(result)
        print(
            f"workers={concurrency} status={result.status} "
            f"elapsed={result.elapsed_s:.1f}s samples={result.samples_written} "
            f"peak_mem={result.peak_memory_used_mb}MiB peak_util={result.peak_gpu_util_percent}%",
            flush=True,
        )
        if result.status != "PASS":
            print(f"Failure reason: {result.failure_reason}", flush=True)
            if args.stop_on_failure:
                break

    write_final_summary(run_dir, args, results)
    print(f"\nReport: {run_dir / 'report.md'}")
    print(f"Summary CSV: {run_dir / 'summary.csv'}")
    print(f"Latest link: {EVAL_ROOT / 'latest'}")
    return 0 if any(item.status == "PASS" for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
