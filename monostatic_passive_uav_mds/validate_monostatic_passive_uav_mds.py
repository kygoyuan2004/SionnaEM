#!/usr/bin/env python3
"""Validation suite for monostatic passive UAV MDS script."""

from __future__ import annotations

import traceback
from pathlib import Path
from typing import Callable

import numpy as np

from rt_monostatic_passive_uav_mds import (
    OUTPUT_LOG_DIR,
    PassiveUavMdsConfig,
    compute_spectrogram,
    run_simulation,
    scatterer_state,
    scatterer_trajectories,
    theory_metrics,
)


REPORT_PATH = OUTPUT_LOG_DIR / "validation_report.txt"


class ValidationFailure(AssertionError):
    pass


def check(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationFailure(message)


def validation_cfg(**kwargs) -> PassiveUavMdsConfig:
    base = dict(
        snapshots=128,
        fs=2000.0,
        stft_win=64,
        max_depth=0,
        add_noise=False,
    )
    base.update(kwargs)
    return PassiveUavMdsConfig(**base)


def test_geometry() -> None:
    cfg = validation_cfg()
    traj = scatterer_trajectories(cfg)
    positions = np.asarray(traj["positions"], dtype=np.float64)
    rotor_centers = np.asarray(traj["rotor_centers"], dtype=np.float64)
    radii = np.asarray(traj["radii"], dtype=np.float64)
    rotor_ids = np.asarray(traj["rotor_ids"], dtype=np.int32)
    t_vec = cfg.t_vec

    expected_body = np.asarray(cfg.body_position0, dtype=np.float64) + np.outer(t_vec, cfg.body_velocity)
    check(np.allclose(positions[:, 0, :], expected_body, atol=1e-10), "body(t) != body0 + V*t")

    offsets = rotor_centers - positions[:, 0:1, :]
    initial_offsets = offsets[0]
    check(np.allclose(offsets, initial_offsets[np.newaxis, :, :], atol=1e-10), "rotor centers do not follow body")

    for scatterer_idx in range(1, cfg.n_scatterers):
        rotor_idx = rotor_ids[scatterer_idx]
        rho = radii[scatterer_idx]
        rel = positions[:, scatterer_idx, :] - rotor_centers[:, rotor_idx, :]
        dist = np.linalg.norm(rel, axis=1)
        check(np.allclose(dist, rho, atol=1e-9), f"blade sample {scatterer_idx} radius mismatch")

    moved = positions[-1] - positions[0]
    body_shift = expected_body[-1] - expected_body[0]
    centers_shift = rotor_centers[-1] - rotor_centers[0]
    check(np.allclose(centers_shift, body_shift[np.newaxis, :], atol=1e-10), "rotor center shift != body shift")
    check(np.linalg.norm(moved[0] - body_shift) < 1e-10, "body final displacement mismatch")


def test_stft_output() -> None:
    cfg = validation_cfg(snapshots=96, stft_win=32)
    signal = np.exp(1j * 2.0 * np.pi * 125.0 * cfg.t_vec)
    f, t, s_db = compute_spectrogram(signal, cfg)
    check(len(f) == cfg.stft_win, "frequency axis length != stft_win")
    check(np.all(np.diff(f) > 0), "frequency axis is not monotonic")
    check(np.isclose(f[len(f) // 2], 0.0), "frequency axis is not zero-centered")
    check(s_db.shape == (len(f), len(t)), "S_dB shape mismatch")
    check(np.all(np.isfinite(s_db)), "S_dB contains non-finite values")
    check(abs(float(np.max(s_db))) < 1e-6, "S_dB max is not normalized to 0 dB")
    check(np.isclose(f[1] - f[0], cfg.fs / cfg.stft_win), "STFT df mismatch")


def test_default_rt_signal_and_doppler() -> None:
    cfg = validation_cfg(snapshots=128, stft_win=64)
    result = run_simulation(cfg, verbose=False)
    signal = result["signal"]
    check(np.all(np.isfinite(signal)), "signal contains non-finite values")
    check(np.max(np.abs(signal)) > 0.0, "signal is all zero")

    f = result["f_stft"]
    s_db = result["S_dB"]
    profile = np.max(s_db, axis=1)
    peak_freq = float(f[int(np.argmax(profile))])
    theory = theory_metrics(cfg)
    df = cfg.fs / cfg.stft_win
    body = theory["body_doppler_hz"]
    check(abs(peak_freq - body) <= df + 1e-9, f"STFT main peak {peak_freq:.2f} Hz not within one bin of body Doppler {body:.2f} Hz")

    active = np.flatnonzero(profile >= -35.0)
    check(len(active) > 0, "no active MDS support bins found")
    support_min = float(f[active[0]])
    support_max = float(f[active[-1]])
    expected_min = theory["f_min_expected_hz"]
    expected_max = theory["f_max_expected_hz"]
    tolerance = 8.0 * df
    check(support_min >= expected_min - tolerance, "MDS support extends too far below theoretical band")
    check(support_max <= expected_max + tolerance, "MDS support extends too far above theoretical band")


def test_phase_continuity() -> None:
    cfg = validation_cfg(snapshots=128, stft_win=64)
    result = run_simulation(cfg, verbose=False)
    phase = np.unwrap(np.angle(result["signal_clean"]))
    dphi = np.diff(phase)
    inst_freq = np.gradient(phase, 1.0 / cfg.fs) / (2.0 * np.pi)
    check(np.all(np.isfinite(phase)), "unwrapped phase contains non-finite values")
    check(np.all(np.abs(dphi) < 0.99 * np.pi), "phase has an apparent aliasing jump")
    check(np.max(np.abs(inst_freq)) < 0.99 * cfg.nyquist, "instantaneous frequency exceeds Nyquist")


def run_ablation(name: str, expected: str, cfg: PassiveUavMdsConfig, predicate: Callable[[dict], None]) -> None:
    result = run_simulation(cfg, verbose=False)
    try:
        predicate(result)
    except Exception as exc:
        raise ValidationFailure(f"{name} failed ({expected}): {exc}") from exc


def test_ablations() -> None:
    def support_width(result: dict, threshold_db: float = -35.0) -> float:
        f = result["f_stft"]
        profile = np.max(result["S_dB"], axis=1)
        active = np.flatnonzero(profile >= threshold_db)
        if len(active) == 0:
            return 0.0
        return float(f[active[-1]] - f[active[0]])

    def peak_frequency(result: dict) -> float:
        profile = np.max(result["S_dB"], axis=1)
        return float(result["f_stft"][int(np.argmax(profile))])

    def dominant_centroid(result: dict, threshold_db: float = -25.0) -> float:
        f = result["f_stft"]
        profile = np.max(result["S_dB"], axis=1)
        active = profile >= threshold_db
        if not np.any(active):
            return peak_frequency(result)
        weights = 10.0 ** (profile[active] / 20.0)
        return float(np.sum(f[active] * weights) / np.sum(weights))

    base = validation_cfg(snapshots=128, stft_win=64)
    df = base.fs / base.stft_win

    run_ablation(
        "Case A no rotation",
        "no micro-Doppler spreading",
        validation_cfg(rotation_freq=0.0, snapshots=128, stft_win=64),
        lambda r: check(support_width(r) <= 4.0 * df, "rotation-free spectrum is unexpectedly wide"),
    )

    run_ablation(
        "Case B no body translation",
        "MDS centered near 0 Hz",
        validation_cfg(body_speed=0.0, snapshots=128, stft_win=64),
        lambda r: check(abs(dominant_centroid(r)) <= 4.0 * df, "zero-body-speed MDS energy is not centered near 0 Hz"),
    )

    run_ablation(
        "Case C blade disabled",
        "only body Doppler line remains",
        validation_cfg(blade_weight=0.0, snapshots=128, stft_win=64),
        lambda r: check(support_width(r) <= 4.0 * df, "body-only spectrum is unexpectedly wide"),
    )

    def body_off_predicate(result: dict) -> None:
        body = theory_metrics(result["config"])["body_doppler_hz"]
        f = result["f_stft"]
        profile = np.max(result["S_dB"], axis=1)
        body_bin = int(np.argmin(np.abs(f - body)))
        check(profile[body_bin] < -3.0, "body Doppler line did not weaken enough when body_weight=0")
        check(support_width(result) > 4.0 * df, "rotor-only spectrum is not visibly spread")

    run_ablation(
        "Case D body disabled",
        "body line weakens and rotor micro-Doppler dominates",
        validation_cfg(body_weight=0.0, snapshots=128, stft_win=64),
        body_off_predicate,
    )


def main() -> None:
    OUTPUT_LOG_DIR.mkdir(parents=True, exist_ok=True)
    tests: list[tuple[str, Callable[[], None]]] = [
        ("geometry", test_geometry),
        ("stft_output", test_stft_output),
        ("default_rt_signal_and_doppler", test_default_rt_signal_and_doppler),
        ("phase_continuity", test_phase_continuity),
        ("ablations", test_ablations),
    ]

    lines = ["Monostatic passive UAV MDS validation", "=" * 48]
    failures = 0
    for name, fn in tests:
        try:
            fn()
            line = f"PASS {name}"
            print(line)
            lines.append(line)
        except Exception as exc:
            failures += 1
            line = f"FAIL {name}: {exc}"
            print(line)
            lines.append(line)
            lines.append(traceback.format_exc())

    if failures:
        lines.append(f"FAILED: {failures} test group(s)")
        REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
        raise SystemExit(1)

    lines.append("ALL TESTS PASSED")
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Validation report: {REPORT_PATH}")


if __name__ == "__main__":
    main()
