#!/usr/bin/env python3
"""Build a UAV micro-Doppler dataset with Sionna RT PathSolver.

This generator mirrors /home/zfh/SionnaEM/dataset, but replaces the previous
geometric LOS channel with actual Sionna RT PathSolver calls. For every RT
snapshot, the UAV body/blade scatterers are represented as passive Receiver
probes, their positions are updated, PathSolver is called, and the one-way
BS-to-probe CIR is coherently summed into a monostatic return.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATASET_SCRIPTS = PROJECT_ROOT / "dataset" / "scripts"
if str(DATASET_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(DATASET_SCRIPTS))

from metadata_utils import METADATA_FIELDS  # noqa: E402
from motion_utils import (  # noqa: E402
    CLASS_SPECS,
    ClassSpec,
    SPEED_OF_LIGHT,
    build_default_weights,
    monostatic_doppler_theory,
    omega_theta_ratio,
    rotation_y,
    scatterer_count,
    scatterer_trajectories,
)
from stft_utils import (  # noqa: E402
    compute_stft,
    frequency_profile_metrics,
    save_stft_image,
    stft_window_metrics,
)


RT_CLASS_SPECS: dict[str, ClassSpec] = {
    "level_v0": CLASS_SPECS["level_v0"],
    "pitch30_v10": CLASS_SPECS["pitch30_v10"],
    "pitch45_v10": CLASS_SPECS["pitch45_v10"],
    "single_blade_v0": ClassSpec("single_blade_v0", 0.0, 0.0),
}


RT_FIELDNAMES = METADATA_FIELDS + [
    "sample_elapsed_s",
    "scene_channel_model",
    "rt_solver",
    "rt_max_depth",
    "rt_los",
    "rt_specular_reflection",
    "rt_diffuse_reflection",
    "rt_diffraction",
    "rt_refraction",
    "rt_snapshot_stride",
    "rt_snapshots_solved",
    "rt_elapsed_s",
    "rt_total_valid_paths",
    "rt_min_valid_paths_per_solved_snapshot",
    "rt_max_valid_paths_per_solved_snapshot",
    "rt_cir_sampling_frequency_hz",
]


TIMING_FIELDNAMES = [
    "sample_id",
    "class_id",
    "num_snapshots",
    "rt_snapshot_stride",
    "rt_snapshots_solved",
    "rt_elapsed_s",
    "sample_elapsed_s",
    "rt_elapsed_per_solved_snapshot_s",
    "created_time",
]


def write_rt_metadata_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RT_FIELDNAMES, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def append_rt_manifest(path: Path, row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, ensure_ascii=False) + "\n")


def write_timing_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=TIMING_FIELDNAMES, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            timing = dict(row)
            solved = max(1, int(float(row.get("rt_snapshots_solved", 1))))
            timing["rt_elapsed_per_solved_snapshot_s"] = float(row.get("rt_elapsed_s", 0.0)) / solved
            writer.writerow(timing)


def parse_scalar(raw: str) -> Any:
    raw = raw.strip()
    if not raw:
        return ""
    if (raw.startswith('"') and raw.endswith('"')) or (raw.startswith("'") and raw.endswith("'")):
        return raw[1:-1]
    lower = raw.lower()
    if lower == "true":
        return True
    if lower == "false":
        return False
    try:
        if any(ch in raw for ch in [".", "e", "E"]):
            return float(raw)
        return int(raw)
    except ValueError:
        return raw


def read_simple_yaml(path: Path) -> dict[str, Any]:
    cfg: dict[str, Any] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        cfg[key.strip()] = parse_scalar(value)
    return cfg


def ensure_dirs(root: Path) -> None:
    for rel in ["database", "reports", "logs"]:
        (root / rel).mkdir(parents=True, exist_ok=True)
    for class_id in RT_CLASS_SPECS:
        (root / "images" / class_id).mkdir(parents=True, exist_ok=True)
        (root / "tensors" / class_id).mkdir(parents=True, exist_ok=True)


def local_now_iso() -> str:
    return datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds")


def add_awgn(signal: np.ndarray, snr_db: float, rng: np.random.Generator) -> np.ndarray:
    power = float(np.mean(np.abs(signal) ** 2))
    if power <= 0.0:
        return signal.copy()
    noise_power = power / (10.0 ** (float(snr_db) / 10.0))
    noise = math.sqrt(noise_power / 2.0) * (
        rng.standard_normal(signal.shape) + 1j * rng.standard_normal(signal.shape)
    )
    return signal + noise


def load_rt_scene(cfg: dict[str, Any]):
    import sionna
    from sionna.rt import PlanarArray, load_scene

    scene_xml = str(cfg.get("scene_xml_path", "") or "").strip()
    if scene_xml:
        scene = load_scene(scene_xml)
        scene_source = scene_xml
    else:
        scene_name = str(cfg.get("scene_name", "floor_wall"))
        scene_obj = getattr(sionna.rt.scene, scene_name, sionna.rt.scene.floor_wall)
        scene = load_scene(scene_obj)
        scene_source = f"sionna.rt.scene.{scene_name}"

    scene.frequency = float(cfg["carrier_frequency_hz"])
    scene.tx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )
    scene.rx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )
    return scene, scene_source


def setup_scene(cfg: dict[str, Any], initial_positions: np.ndarray, labels: list[str]):
    from sionna.rt import Receiver, Transmitter

    scene, scene_source = load_rt_scene(cfg)
    bs = [
        float(cfg["bs_position_x"]),
        float(cfg["bs_position_y"]),
        float(cfg["bs_position_z"]),
    ]
    scene.add(
        Transmitter(
            name=str(cfg.get("scene_tx_name", "bs_radar")),
            position=bs,
            orientation=[0.0, 0.0, 0.0],
            power_dbm=0.0,
        )
    )

    probe_names = []
    for idx, (label, pos) in enumerate(zip(labels, initial_positions)):
        name = f"probe_{idx:03d}_{label}".replace(".", "p")
        scene.add(
            Receiver(
                name=name,
                position=np.asarray(pos, dtype=np.float64).tolist(),
                orientation=[0.0, 0.0, 0.0],
            )
        )
        probe_names.append(name)
    return scene, scene_source, probe_names


def update_probe_positions(scene: Any, probe_names: list[str], positions: np.ndarray) -> None:
    for name, pos in zip(probe_names, positions):
        scene.get(name).position = np.asarray(pos, dtype=np.float64).tolist()


def run_solver(scene: Any, solver: Any, cfg: dict[str, Any]) -> np.ndarray:
    paths = solver(
        scene,
        max_depth=int(cfg.get("rt_max_depth", 2)),
        los=bool(cfg.get("rt_los", True)),
        specular_reflection=bool(cfg.get("rt_specular_reflection", True)),
        diffuse_reflection=bool(cfg.get("rt_diffuse_reflection", False)),
        diffraction=bool(cfg.get("rt_diffraction", False)),
        edge_diffraction=bool(cfg.get("rt_edge_diffraction", False)),
        refraction=bool(cfg.get("rt_refraction", False)),
        synthetic_array=bool(cfg.get("rt_synthetic_array", False)),
    )
    a, _ = paths.cir(
        sampling_frequency=float(cfg.get("rt_cir_sampling_frequency_hz", 122.88e6)),
        normalize_delays=False,
        out_type="numpy",
    )
    return np.asarray(a, dtype=np.complex128)


def one_way_channels_by_rx(a: np.ndarray, n_receivers: int) -> tuple[np.ndarray, np.ndarray]:
    if a.ndim != 6:
        raise ValueError(f"Expected 6-D CIR coefficient tensor, got shape {a.shape}")
    if a.shape[0] < n_receivers:
        raise ValueError(f"Expected at least {n_receivers} receivers, got {a.shape[0]}")
    coeff = a[:n_receivers, 0, 0, 0, :, 0]
    valid = np.abs(coeff) > 0.0
    return np.sum(coeff, axis=-1), np.sum(valid, axis=-1).astype(np.int32)


def interp_complex(solved_indices: np.ndarray, solved_values: np.ndarray, n: int) -> np.ndarray:
    full_idx = np.arange(n, dtype=np.float64)
    out = np.empty((n, solved_values.shape[1]), dtype=np.complex128)
    for k in range(solved_values.shape[1]):
        real = np.interp(full_idx, solved_indices.astype(np.float64), solved_values[:, k].real)
        imag = np.interp(full_idx, solved_indices.astype(np.float64), solved_values[:, k].imag)
        out[:, k] = real + 1j * imag
    return out


def single_blade_tip_trajectory(
    *,
    num_snapshots: int,
    sampling_rate_hz: float,
    rotor_center: np.ndarray,
    body_velocity0: np.ndarray,
    degree_rad: float,
    rotor_frequency_hz: float,
    blade_radius_m: float,
    initial_phase: float,
) -> dict[str, Any]:
    """Generate one rotating blade-tip scatterer for the clean v=0 RT class."""

    t = np.arange(int(num_snapshots), dtype=np.float64) / float(sampling_rate_hz)
    center0 = np.asarray(rotor_center, dtype=np.float64)
    body_velocity0 = np.asarray(body_velocity0, dtype=np.float64)
    centers = center0[None, :] + t[:, None] * body_velocity0[None, :]

    omega = 2.0 * np.pi * float(rotor_frequency_hz)
    phi = omega * t + float(initial_phase)
    rho = float(blade_radius_m)
    r_body_to_world = rotation_y(degree_rad)

    rel_local = np.stack([rho * np.cos(phi), rho * np.sin(phi), np.zeros_like(t)], axis=1)
    vel_local = np.stack([-omega * rho * np.sin(phi), omega * rho * np.cos(phi), np.zeros_like(t)], axis=1)
    rel_world = rel_local @ r_body_to_world.T
    vel_world = vel_local @ r_body_to_world.T

    positions = (centers[:, None, :] + rel_world[:, None, :]).astype(np.float32)
    velocities = (body_velocity0[None, None, :] + vel_world[:, None, :]).astype(np.float32)
    rotor_centers = centers[:, None, :].astype(np.float32)

    return {
        "t": t.astype(np.float32),
        "positions": positions,
        "velocities": velocities,
        "rotor_centers": rotor_centers,
        "labels": ["single_blade_tip"],
        "radii": np.asarray([rho], dtype=np.float32),
        "scatterer_type_codes": np.asarray([1], dtype=np.int32),
        "rotor_ids": np.asarray([0], dtype=np.int32),
    }


def generate_case(
    *,
    cfg: dict[str, Any],
    root: Path,
    class_id: str,
    sample_index: int,
    snapshot_override: int | None = None,
) -> dict[str, Any]:
    sample_start = time.perf_counter()
    spec = RT_CLASS_SPECS[class_id]
    seed = int(cfg["random_seed"]) + sample_index + 100000 * list(RT_CLASS_SPECS).index(class_id)
    rng = np.random.default_rng(seed)

    num_snapshots = int(snapshot_override or cfg["num_snapshots"])
    fs = float(cfg["sampling_rate_hz"])
    rotor_frequency = float(cfg["rotor_frequency_hz"]) * rng.uniform(
        1.0 - float(cfg.get("rotor_frequency_jitter_fraction", 0.0)),
        1.0 + float(cfg.get("rotor_frequency_jitter_fraction", 0.0)),
    )
    blade_radius = float(cfg["blade_radius_m"]) * rng.uniform(
        1.0 - float(cfg.get("blade_radius_jitter_fraction", 0.0)),
        1.0 + float(cfg.get("blade_radius_jitter_fraction", 0.0)),
    )
    noise_snr = rng.uniform(float(cfg["noise_snr_min_db"]), float(cfg["noise_snr_max_db"]))
    position_jitter = float(cfg.get("position_jitter_m", 0.0))
    body_position0 = np.asarray(
        [
            float(cfg["body_position_x"]),
            float(cfg["body_position_y"]),
            float(cfg["body_position_z"]),
        ],
        dtype=np.float64,
    )
    if position_jitter:
        body_position0 += rng.uniform(-position_jitter, position_jitter, size=3)
    body_velocity0 = np.asarray([float(spec.body_speed_m_s), 0.0, 0.0], dtype=np.float64)
    body_acceleration = np.zeros(3, dtype=np.float64)
    degree_rad = math.radians(float(spec.degree))
    if class_id == "single_blade_v0":
        num_rotors = 1
        num_blades = 1
        points_per_blade = 1
    else:
        num_rotors = int(cfg["num_rotors"])
        num_blades = int(cfg["num_blades_per_rotor"])
        points_per_blade = int(cfg["points_per_blade"])
    initial_phase = rng.uniform(0.0, 2.0 * np.pi)
    blade_phase_offsets = rng.uniform(-0.08, 0.08, size=(num_rotors, num_blades))

    if class_id == "single_blade_v0":
        traj = single_blade_tip_trajectory(
            num_snapshots=num_snapshots,
            sampling_rate_hz=fs,
            rotor_center=body_position0,
            body_velocity0=body_velocity0,
            degree_rad=degree_rad,
            rotor_frequency_hz=rotor_frequency,
            blade_radius_m=blade_radius,
            initial_phase=initial_phase,
        )
        weights = np.asarray([1.0], dtype=np.float64)
        n_scatterers = 1
    else:
        traj = scatterer_trajectories(
            num_snapshots=num_snapshots,
            sampling_rate_hz=fs,
            body_position0=body_position0,
            body_velocity0=body_velocity0,
            body_acceleration=body_acceleration,
            degree_rad=degree_rad,
            rotor_frequency_hz=rotor_frequency,
            blade_radius_m=blade_radius,
            rotor_arm_length_m=float(cfg["rotor_arm_length_m"]),
            num_rotors=num_rotors,
            num_blades_per_rotor=num_blades,
            points_per_blade=points_per_blade,
            initial_rotor_phase=initial_phase,
            blade_phase_offsets=blade_phase_offsets,
        )
        weights = build_default_weights(
            num_rotors,
            num_blades,
            points_per_blade,
            rng,
            float(cfg.get("scatterer_weight_jitter_fraction", 0.0)),
        )
        n_scatterers = scatterer_count(num_rotors, num_blades, points_per_blade)

    scene, scene_source, probe_names = setup_scene(cfg, traj["positions"][0], traj["labels"])
    from sionna.rt import PathSolver

    solver = PathSolver()
    stride = max(1, int(cfg.get("rt_snapshot_stride", 1)))
    solved_indices = np.arange(0, num_snapshots, stride, dtype=np.int32)
    if solved_indices[-1] != num_snapshots - 1:
        solved_indices = np.append(solved_indices, num_snapshots - 1)

    h_solved = np.zeros((len(solved_indices), n_scatterers), dtype=np.complex128)
    path_counts_solved = np.zeros((len(solved_indices), n_scatterers), dtype=np.int32)
    start = time.perf_counter()
    for j, snap_idx in enumerate(solved_indices):
        update_probe_positions(scene, probe_names, traj["positions"][int(snap_idx)])
        a = run_solver(scene, solver, cfg)
        h, counts = one_way_channels_by_rx(a, n_scatterers)
        h_solved[j] = h
        path_counts_solved[j] = counts
        if (j + 1) % max(1, len(solved_indices) // 4) == 0:
            print(f"  {class_id}_{sample_index:04d}: RT {j + 1}/{len(solved_indices)} solved snapshots", flush=True)
    rt_elapsed = time.perf_counter() - start

    if len(solved_indices) == num_snapshots:
        csi_one_way = h_solved
        path_counts = path_counts_solved
    else:
        csi_one_way = interp_complex(solved_indices, h_solved, num_snapshots)
        path_counts = np.rint(
            np.vstack([
                np.interp(np.arange(num_snapshots), solved_indices, path_counts_solved[:, k])
                for k in range(n_scatterers)
            ]).T
        ).astype(np.int32)

    return_per_scatterer = weights[None, :] * csi_one_way * csi_one_way
    signal_clean = np.sum(return_per_scatterer, axis=1)
    signal_noisy = add_awgn(signal_clean, noise_snr, rng)
    f_axis, t_axis, s_complex, s_db = compute_stft(
        signal_noisy,
        fs,
        int(cfg["stft_window_size"]),
        int(cfg["stft_overlap"]),
        int(cfg["stft_nfft"]),
    )

    bs_position = np.asarray([float(cfg["bs_position_x"]), float(cfg["bs_position_y"]), float(cfg["bs_position_z"])])
    freqs = monostatic_doppler_theory(
        traj["positions"],
        traj["velocities"],
        bs_position,
        float(cfg["carrier_frequency_hz"]),
    )
    body_freq = float(np.median(freqs[:, 0]))
    f_min_theory = float(np.min(freqs))
    f_max_theory = float(np.max(freqs))
    micro_max = float(max(abs(f_min_theory - body_freq), abs(f_max_theory - body_freq)))
    window = stft_window_metrics(
        rotor_frequency_hz=rotor_frequency,
        micro_doppler_max_hz=micro_max,
        sampling_rate_hz=fs,
        stft_window_size=int(cfg["stft_window_size"]),
    )
    observed = frequency_profile_metrics(f_axis, s_db)

    ratio = omega_theta_ratio(degree_rad)
    omega = float(2.0 * np.pi * rotor_frequency)
    omega_h = omega / ratio
    omega_theta = omega
    sample_id = f"{class_id}_{sample_index:04d}"
    image_rel = Path("images") / class_id / f"{sample_id}.png"
    tensor_rel = Path("tensors") / class_id / f"{sample_id}.npz"

    metadata = {
        "sample_id": sample_id,
        "class_id": class_id,
        "image_path": str(image_rel),
        "tensor_path": str(tensor_rel),
        "scene_name": str(cfg["scene_name"]),
        "carrier_frequency_hz": float(cfg["carrier_frequency_hz"]),
        "sampling_rate_hz": fs,
        "num_snapshots": num_snapshots,
        "snapshot_duration_s": num_snapshots / fs,
        "stft_window_size": int(cfg["stft_window_size"]),
        "stft_overlap": int(cfg["stft_overlap"]),
        "stft_nfft": int(cfg["stft_nfft"]),
        "stft_window_duration_s": window["stft_window_duration_s"],
        "stft_frequency_resolution_hz": window["stft_frequency_resolution_hz"],
        "rotor_frequency_hz": rotor_frequency,
        "rotor_period_s": window["rotor_period_s"],
        "omega_rad_s": omega,
        "omega_h_rad_s": omega_h,
        "omega_theta_rad_s": omega_theta,
        "omega_theta_over_omega_h": ratio,
        "degree": float(spec.degree),
        "degree_rad": degree_rad,
        "body_speed_m_s": float(spec.body_speed_m_s),
        "body_velocity_x": float(body_velocity0[0]),
        "body_velocity_y": float(body_velocity0[1]),
        "body_velocity_z": float(body_velocity0[2]),
        "body_acceleration_x": 0.0,
        "body_acceleration_y": 0.0,
        "body_acceleration_z": 0.0,
        "blade_radius_m": blade_radius,
        "num_uavs": int(cfg["num_uavs"]),
        "num_rotors": num_rotors,
        "num_blades_per_rotor": num_blades,
        "points_per_blade": points_per_blade,
        "num_scatterers": n_scatterers,
        "doppler_body_theory_hz": body_freq,
        "micro_doppler_max_theory_hz": micro_max,
        "doppler_support_min_theory_hz": f_min_theory,
        "doppler_support_max_theory_hz": f_max_theory,
        "stft_support_min_observed_hz": observed["stft_support_min_observed_hz"],
        "stft_support_max_observed_hz": observed["stft_support_max_observed_hz"],
        "stft_peak_observed_hz": observed["stft_peak_observed_hz"],
        "window_ratio_Twin_over_Trot": window["window_ratio_Twin_over_Trot"],
        "window_strict_metric": window["window_strict_metric"],
        "window_check_pass": window["window_check_pass"],
        "random_seed": seed,
        "noise_snr_db": noise_snr,
        "created_time": local_now_iso(),
        "initial_body_position_x": float(body_position0[0]),
        "initial_body_position_y": float(body_position0[1]),
        "initial_body_position_z": float(body_position0[2]),
        "initial_rotor_phase_rad": float(initial_phase),
        "bs_position_x": float(cfg["bs_position_x"]),
        "bs_position_y": float(cfg["bs_position_y"]),
        "bs_position_z": float(cfg["bs_position_z"]),
        "scene_channel_model": "sionna_rt_pathsolver",
        "rt_solver": "sionna.rt.PathSolver",
        "rt_max_depth": int(cfg.get("rt_max_depth", 2)),
        "rt_los": bool(cfg.get("rt_los", True)),
        "rt_specular_reflection": bool(cfg.get("rt_specular_reflection", True)),
        "rt_diffuse_reflection": bool(cfg.get("rt_diffuse_reflection", False)),
        "rt_diffraction": bool(cfg.get("rt_diffraction", False)),
        "rt_refraction": bool(cfg.get("rt_refraction", False)),
        "rt_snapshot_stride": stride,
        "rt_snapshots_solved": int(len(solved_indices)),
        "rt_elapsed_s": rt_elapsed,
        "rt_total_valid_paths": int(np.sum(path_counts_solved)),
        "rt_min_valid_paths_per_solved_snapshot": int(np.min(np.sum(path_counts_solved, axis=1))),
        "rt_max_valid_paths_per_solved_snapshot": int(np.max(np.sum(path_counts_solved, axis=1))),
        "rt_cir_sampling_frequency_hz": float(cfg.get("rt_cir_sampling_frequency_hz", 122.88e6)),
    }

    save_stft_image(
        s_db,
        f_axis,
        root / image_rel,
        f_min_hz=float(cfg["doppler_plot_min_hz"]),
        f_max_hz=float(cfg["doppler_plot_max_hz"]),
        dynamic_range_db=float(cfg["stft_dynamic_range_db"]),
    )
    np.savez_compressed(
        root / tensor_rel,
        signal_clean=signal_clean,
        signal_noisy=signal_noisy,
        csi_one_way=csi_one_way,
        return_per_scatterer=return_per_scatterer,
        S_complex=s_complex,
        S_dB=s_db,
        f_axis=f_axis,
        t_axis=t_axis,
        positions=traj["positions"],
        velocities=traj["velocities"],
        rotor_centers=traj["rotor_centers"],
        monostatic_freq_theory=freqs,
        weights=weights.astype(np.float32),
        radii=traj["radii"],
        scatterer_type_codes=traj["scatterer_type_codes"],
        rotor_ids=traj["rotor_ids"],
        blade_phase_offsets=blade_phase_offsets.astype(np.float32),
        path_counts=path_counts,
        rt_solved_indices=solved_indices,
        rt_csi_one_way_solved=h_solved,
        rt_path_counts_solved=path_counts_solved,
        metadata_json=np.asarray(json.dumps(metadata, ensure_ascii=False)),
        labels_json=np.asarray(json.dumps(traj["labels"], ensure_ascii=False)),
        scene_source=np.asarray(scene_source),
    )
    metadata["sample_elapsed_s"] = time.perf_counter() - sample_start
    return metadata


def write_metadata_md(root: Path, rows: list[dict[str, Any]], cfg: dict[str, Any]) -> None:
    counts = {class_id: sum(1 for row in rows if row["class_id"] == class_id) for class_id in RT_CLASS_SPECS}
    lines = [
        "# Sionna RT UAV STFT Dataset Metadata Summary",
        "",
        f"- Total samples: `{len(rows)}`",
        "- Channel model: `sionna_rt_pathsolver`",
        "- Important: this dataset calls `sionna.rt.PathSolver` for every configured RT snapshot.",
        f"- RT snapshot stride: `{cfg.get('rt_snapshot_stride', 1)}`",
        f"- RT max depth: `{cfg.get('rt_max_depth', 2)}`",
        "",
        "## Class Counts",
        "",
        "| class_id | samples | degree | body speed [m/s] |",
        "| --- | ---: | ---: | ---: |",
    ]
    for class_id, spec in RT_CLASS_SPECS.items():
        lines.append(f"| `{class_id}` | {counts[class_id]} | {spec.degree} | {spec.body_speed_m_s} |")
    if rows:
        lines.extend(["", "## First Sample Snapshot", ""])
        for key in RT_FIELDNAMES:
            if key in rows[0]:
                lines.append(f"- `{key}`: `{rows[0][key]}`")
    (root / "database" / "metadata.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate UAV STFT samples using Sionna RT PathSolver.")
    parser.add_argument("--root", type=Path, default=PROJECT_ROOT / "RT_dataset")
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "RT_dataset" / "configs" / "rt_dataset_config.yaml")
    parser.add_argument("--samples-per-class", type=int, default=None)
    parser.add_argument("--classes", nargs="*", default=list(RT_CLASS_SPECS))
    parser.add_argument("--snapshot-override", type=int, default=None, help="Testing only: override num_snapshots.")
    parser.add_argument("--rt-snapshot-stride", type=int, default=None)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    cfg = read_simple_yaml(args.config)
    if args.samples_per_class is not None:
        cfg["samples_per_class"] = int(args.samples_per_class)
    if args.rt_snapshot_stride is not None:
        cfg["rt_snapshot_stride"] = int(args.rt_snapshot_stride)

    root = args.root
    ensure_dirs(root)
    manifest_path = root / "database" / "manifest.jsonl"
    metadata_path = root / "database" / "metadata.csv"
    timing_path = root / "database" / "timing.csv"

    rows: list[dict[str, Any]] = []
    if args.resume and metadata_path.exists():
        with metadata_path.open("r", encoding="utf-8", newline="") as f:
            rows.extend(csv.DictReader(f))
    done = {row["sample_id"] for row in rows}
    if not args.resume:
        manifest_path.write_text("", encoding="utf-8")

    total = int(cfg["samples_per_class"]) * len(args.classes)
    completed = len(done)
    for class_id in args.classes:
        if class_id not in RT_CLASS_SPECS:
            raise ValueError(f"Unknown class: {class_id}")
        for sample_index in range(int(cfg["samples_per_class"])):
            sample_id = f"{class_id}_{sample_index:04d}"
            if sample_id in done:
                continue
            print(f"[{completed + 1}/{total}] generating {sample_id}", flush=True)
            row = generate_case(
                cfg=cfg,
                root=root,
                class_id=class_id,
                sample_index=sample_index,
                snapshot_override=args.snapshot_override,
            )
            rows.append(row)
            append_rt_manifest(manifest_path, row)
            write_rt_metadata_csv(metadata_path, rows)
            write_timing_csv(timing_path, rows)
            write_metadata_md(root, rows, cfg)
            completed += 1

    write_rt_metadata_csv(metadata_path, rows)
    write_timing_csv(timing_path, rows)
    write_metadata_md(root, rows, cfg)
    print(f"Done. Samples: {len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
