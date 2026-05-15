#!/usr/bin/env python3
"""Micro-Doppler Integration Demo — Sionna RT CIR pipeline with MicroDopplerModulator.

Step 5: Proves that MicroDopplerModulator can be embedded into the Sionna RT
CIR generation pipeline with a single RT solve per scene.

Pipeline:
  1. Load scene & run static RT (once per scene)
  2. Apply MicroDopplerModulator to static CIR
  3. Reconstruct time-varying CFR & compute spectrogram
  4. Compare with analytic (scatterer-model) pipeline
  5. Report performance benchmarks

Scenes validated:
  - Paris etoile, LOS only (free-space baseline)
  - Paris etoile, multipath (depth=5)
  - floor_wall, simplified scene (depth=5)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from scipy.constants import c as SPEED_OF_LIGHT

from micro_doppler_modulator import UAVMicroDopplerConfig, MicroDopplerModulator

import sionna
from sionna.rt import load_scene, PathSolver, PlanarArray, Receiver, Transmitter

OUT_DIR = Path(__file__).resolve().parent.parent / "figures"
OUT_DIR.mkdir(exist_ok=True)

# ══════════════════════════════════════════════════════════════════════════════
# Scene definitions
# ══════════════════════════════════════════════════════════════════════════════

SCENE_CONFIGS = {
    "paris_los": {
        "scene_fn": sionna.rt.scene.etoile,
        "body_pos": [0.0, 0.0, 10.0],
        "radar_pos": [-40.0, 0.0, 10.0],
        "max_depth": 0,
        "label": "Paris etoile — Free Space (LOS only)",
    },
    "paris_multipath": {
        "scene_fn": sionna.rt.scene.etoile,
        "body_pos": [0.0, 0.0, 10.0],
        "radar_pos": [-40.0, 0.0, 10.0],
        "max_depth": 5,
        "label": "Paris etoile — Multipath",
    },
    "floor_wall": {
        "scene_fn": sionna.rt.scene.floor_wall,
        "body_pos": [0.0, 0.0, 3.0],
        "radar_pos": [-20.0, 0.0, 3.0],
        "max_depth": 5,
        "label": "floor_wall — Simplified Scene",
    },
}

# ══════════════════════════════════════════════════════════════════════════════
# Shared modulator config (matches Step 3 parameters)
# ══════════════════════════════════════════════════════════════════════════════

MODULATOR_CFG = dict(
    carrier_freq=28e9,
    n_rotors=4,
    n_blades_per_rotor=2,
    blade_radius=0.15,
    rotation_freq=100.0,
    rotor_arm_length=0.175,
    body_amplitude=1.0,
    blade_amplitude=0.12,
    radar_mode="monostatic",
    body_velocity=(5.0, 0.0, 0.0),
    sampling_rate=50e3,
    obs_duration=0.2,
    snr_db=30.0,
)


# ══════════════════════════════════════════════════════════════════════════════
# Helper: CIR → baseband signal reconstruction
# ══════════════════════════════════════════════════════════════════════════════

def cir_to_baseband(a_dynamic: np.ndarray) -> np.ndarray:
    """Sum dynamic CIR taps across paths to produce a baseband signal.

    a_dynamic: [n_t, n_rx, n_tx, n_tx_a, n_paths, n_rx_a]
    Returns:  [n_t] complex baseband signal
    """
    # For SISO: n_rx=n_tx=n_tx_a=n_rx_a=1
    a_flat = a_dynamic[:, 0, 0, 0, :, 0]  # [n_t, n_paths]
    return np.sum(a_flat, axis=-1)          # [n_t]


# ══════════════════════════════════════════════════════════════════════════════
# Core: run integrated pipeline for one scene
# ══════════════════════════════════════════════════════════════════════════════

def run_integrated_pipeline(
    scene_key: str,
    modulator: MicroDopplerModulator,
    t_vec: np.ndarray,
) -> dict:
    """Run the full integrated pipeline: RT → static CIR → modulate → CFR.

    Returns dict with signals, spectrograms, timing, and metadata.
    """
    scfg = SCENE_CONFIGS[scene_key]
    cfg = modulator.cfg

    print(f"\n{'─'*60}")
    print(f"  Integrated Pipeline: {scfg['label']}")
    print(f"{'─'*60}")

    # ── 1. Load scene & set up TX/RX ────────────────────────────────────────
    t0 = time.perf_counter()
    scene = load_scene(scfg["scene_fn"])
    scene.frequency = cfg.carrier_freq
    scene.tx_array = PlanarArray(num_rows=1, num_cols=1, vertical_spacing=0.5,
                                  horizontal_spacing=0.5, pattern="iso", polarization="V")
    scene.rx_array = PlanarArray(num_rows=1, num_cols=1, vertical_spacing=0.5,
                                  horizontal_spacing=0.5, pattern="iso", polarization="V")

    body_pos = np.array(scfg["body_pos"], dtype=np.float32)
    radar_pos = np.array(scfg["radar_pos"], dtype=np.float32)

    scene.add(Transmitter(name="uav_tx", position=body_pos,
                          orientation=[0, 0, 0], power_dbm=0.0))
    scene.add(Receiver(name="radar_rx", position=radar_pos,
                        orientation=[0, 0, 0]))

    # ── 2. Single RT solve ──────────────────────────────────────────────────
    solver = PathSolver()
    t_rt_start = time.perf_counter()
    paths = solver(
        scene, max_depth=scfg["max_depth"],
        los=True, specular_reflection=True, diffuse_reflection=True,
        diffraction=False, edge_diffraction=False, refraction=False,
        synthetic_array=False,
    )
    t_rt = time.perf_counter() - t_rt_start

    a_static, tau_static = paths.cir(
        sampling_frequency=122.88e6, normalize_delays=False, out_type="numpy",
    )
    a_static = np.asarray(a_static, dtype=np.complex128)
    tau_static = np.asarray(tau_static, dtype=np.float64)

    n_paths = a_static.shape[4]
    total_power = float(np.sum(np.abs(a_static[0, 0, 0, 0, :, 0]) ** 2))
    print(f"  RT solve: {t_rt*1e3:.1f} ms  |  {n_paths} paths  |  power={total_power:.3e}")

    # ── 3. Micro-Doppler modulation ─────────────────────────────────────────
    t_mod_start = time.perf_counter()
    a_dynamic, tau_dynamic = modulator.modulate_cir(a_static, tau_static, t_vec)
    t_mod = time.perf_counter() - t_mod_start

    # ── 4. CIR → baseband signal ────────────────────────────────────────────
    signal_integrated = cir_to_baseband(a_dynamic)

    # ── 5. STFT spectrogram ─────────────────────────────────────────────────
    t_stft_start = time.perf_counter()
    f_stft, t_stft_arr, S_dB = modulator.stft_spectrogram(
        signal_integrated, n_win=2048, overlap_ratio=0.75,
    )
    t_stft_elapsed = time.perf_counter() - t_stft_start

    t_total = time.perf_counter() - t0

    print(f"  modulate_cir: {t_mod*1e3:.1f} ms  |  STFT: {t_stft_elapsed*1e3:.1f} ms")
    print(f"  Total pipeline: {t_total*1e3:.0f} ms")

    return {
        "scene_key": scene_key,
        "label": scfg["label"],
        "signal": np.asarray(signal_integrated),
        "a_dynamic": np.asarray(a_dynamic),
        "a_static": np.asarray(a_static),
        "tau_static": np.asarray(tau_static),
        "f_stft": np.atleast_1d(np.asarray(f_stft, dtype=np.float64)),
        "t_stft": np.atleast_1d(np.asarray(t_stft_arr, dtype=np.float64)),
        "S_dB": np.atleast_2d(np.asarray(S_dB, dtype=np.float64)),
        "n_paths": n_paths,
        "total_power": total_power,
        "t_rt": t_rt,
        "t_mod": t_mod,
        "t_stft_elapsed": t_stft_elapsed,
        "t_total": t_total,
    }


# ══════════════════════════════════════════════════════════════════════════════
# Core: run analytic pipeline (standalone scatterer model)
# ══════════════════════════════════════════════════════════════════════════════

def run_analytic_pipeline(
    modulator: MicroDopplerModulator,
    t_vec: np.ndarray,
) -> dict:
    """Run the pure analytic pipeline (no RT dependency)."""
    cfg = modulator.cfg
    print(f"\n{'─'*60}")
    print(f"  Analytic Pipeline (scatterer model)")
    print(f"{'─'*60}")

    t0 = time.perf_counter()
    signal = modulator.generate_received_signal(t_vec, add_noise=True)
    t_gen = time.perf_counter() - t0

    f_stft, t_stft, S_dB = modulator.stft_spectrogram(
        signal, n_win=2048, overlap_ratio=0.75,
    )
    t_total = time.perf_counter() - t0

    harmonics = modulator.detect_harmonics(signal)

    # Extract key metrics
    body_doppler = -2.0 * cfg.carrier_freq * cfg.body_velocity[0] / SPEED_OF_LIGHT
    f_dev = modulator.f_dev_peak

    print(f"  Signal generation: {t_gen*1e3:.1f} ms")
    print(f"  Body Doppler: {body_doppler:.0f} Hz  |  f_dev_peak: ±{f_dev*1e-3:.1f} kHz")
    print(f"  Harmonics detected: {len(harmonics)}")
    print(f"  Total: {t_total*1e3:.0f} ms")

    return {
        "signal": signal,
        "f_stft": f_stft,
        "t_stft": t_stft,
        "S_dB": S_dB,
        "harmonics": harmonics,
        "body_doppler": body_doppler,
        "f_dev_peak": f_dev,
        "t_total": t_total,
    }


# ══════════════════════════════════════════════════════════════════════════════
# Validation: compare integrated vs analytic spectrograms
# ══════════════════════════════════════════════════════════════════════════════

def validate_spectrograms(
    integrated: dict,
    analytic: dict,
    scene_key: str,
) -> dict:
    """Compare integrated and analytic pipeline spectrograms.

    Returns dict of validation metrics.
    """
    print(f"\n  --- Validation: {scene_key} ---")

    # ── Spectral spread (energy beyond body-only Doppler) ───────────────────
    f_body = analytic["body_doppler"]
    f = np.atleast_1d(np.asarray(integrated["f_stft"], dtype=np.float64))
    S_int = np.atleast_2d(np.asarray(integrated["S_dB"], dtype=np.float64))
    S_ana = np.atleast_2d(np.asarray(analytic["S_dB"], dtype=np.float64))

    # Time-averaged power spectrum
    p_int = np.mean(10 ** (S_int / 20), axis=1)
    p_ana = np.mean(10 ** (S_ana / 20), axis=1)

    # Band energy: fraction within ±f_dev_peak around body Doppler
    f_dev = analytic["f_dev_peak"]
    band_mask = (f > f_body - f_dev * 1.1) & (f < f_body + f_dev * 1.1)
    int_band_energy = np.sum(p_int[band_mask]) / np.sum(p_int)
    ana_band_energy = np.sum(p_ana[band_mask]) / np.sum(p_ana)

    # Spectral centroid
    centroid_int = np.sum(f * p_int) / np.sum(p_int)
    centroid_ana = np.sum(f * p_ana) / np.sum(p_ana)

    # Correlation between Doppler profiles
    # Interpolate analytic to integrated frequency grid if needed
    corr = np.corrcoef(p_int, p_ana)[0, 1]

    # Spread metric: fraction of energy outside ±200 Hz of body Doppler
    narrow_mask = np.abs(f - f_body) < 200
    spread_int = 1 - np.sum(p_int[narrow_mask]) / np.sum(p_int)
    spread_ana = 1 - np.sum(p_ana[narrow_mask]) / np.sum(p_ana)

    results = {
        "int_band_energy": int_band_energy,
        "ana_band_energy": ana_band_energy,
        "centroid_int": centroid_int,
        "centroid_ana": centroid_ana,
        "correlation": corr,
        "spread_int": spread_int,
        "spread_ana": spread_ana,
    }

    print(f"    Band energy  (integrated): {int_band_energy*100:.1f}%")
    print(f"    Band energy  (analytic):   {ana_band_energy*100:.1f}%")
    print(f"    Centroid     (integrated): {centroid_int:.0f} Hz")
    print(f"    Centroid     (analytic):   {centroid_ana:.0f} Hz")
    print(f"    Correlation:               {corr:.4f}")
    print(f"    Spread       (integrated): {spread_int*100:.1f}%")
    print(f"    Spread       (analytic):   {spread_ana*100:.1f}%")

    return results


# ══════════════════════════════════════════════════════════════════════════════
# Visualization
# ══════════════════════════════════════════════════════════════════════════════

def _ensure_arrays(integrated_results, analytic):
    """Convert STFT outputs to well-formed numpy arrays for safe plotting."""
    f_int = np.atleast_1d(np.asarray(integrated_results["f_stft"], dtype=np.float64))
    t_int = np.atleast_1d(np.asarray(integrated_results["t_stft"], dtype=np.float64))
    S_int = np.atleast_2d(np.asarray(integrated_results["S_dB"], dtype=np.float64))

    f_ana = np.atleast_1d(np.asarray(analytic["f_stft"], dtype=np.float64))
    t_ana = np.atleast_1d(np.asarray(analytic["t_stft"], dtype=np.float64))
    S_ana = np.atleast_2d(np.asarray(analytic["S_dB"], dtype=np.float64))

    return f_int, t_int, S_int, f_ana, t_ana, S_ana


def plot_comparison_figure(
    integrated_results: dict,
    analytic: dict,
    validation: dict,
    modulator: MicroDopplerModulator,
    output_name: str,
):
    """Generate a 2x3 comparison figure for one scene."""
    fig = plt.figure(figsize=(18, 11))
    cfg = modulator.cfg
    f_body = analytic["body_doppler"]
    f_dev = analytic["f_dev_peak"]

    fig.suptitle(
        f"Micro-Doppler Integration Validation — {integrated_results['label']}\n"
        f"fc={cfg.carrier_freq/1e9:.0f} GHz  R={cfg.blade_radius:.2f}m  "
        f"f_rot={cfg.rotation_freq} Hz  beta={modulator.beta:.0f}  "
        f"v_body={cfg.body_velocity[0]:.0f} m/s  SNR={cfg.snr_db:.0f} dB  "
        f"RT: {integrated_results['t_rt']*1e3:.0f}ms  mod: {integrated_results['t_mod']*1e3:.1f}ms  "
        f"paths={integrated_results['n_paths']}",
        fontsize=10, fontweight="bold",
    )

    gs = GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)
    vmin = -40

    f_int, t_int, S_int, f_ana, t_ana, S_ana = _ensure_arrays(integrated_results, analytic)

    f_khz_int = f_int * 1e-3
    t_ms_int = t_int * 1e3
    f_khz_ana = f_ana * 1e-3
    t_ms_ana = t_ana * 1e3

    # (a) Integrated pipeline spectrogram
    ax = fig.add_subplot(gs[0, 0])
    im = ax.pcolormesh(t_ms_int, f_khz_int, S_int,
                        shading="gouraud", vmin=vmin, vmax=0, cmap="inferno")
    ax.axhline(y=f_body * 1e-3, color="cyan", linestyle="--", linewidth=0.8,
               label=f"Body Doppler = {f_body:.0f} Hz")
    ax.set_xlabel("Time [ms]"); ax.set_ylabel("Doppler Frequency [kHz]")
    ax.set_title("(a) Integrated Pipeline\nRT CIR -> Modulator -> Spectrogram")
    ax.legend(fontsize=7)
    plt.colorbar(im, ax=ax, label="Normalized Power [dB]", shrink=0.85)

    # (b) Analytic pipeline spectrogram
    ax = fig.add_subplot(gs[0, 1])
    im = ax.pcolormesh(t_ms_ana, f_khz_ana, S_ana,
                        shading="gouraud", vmin=vmin, vmax=0, cmap="inferno")
    ax.axhline(y=f_body * 1e-3, color="cyan", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time [ms]"); ax.set_ylabel("Doppler Frequency [kHz]")
    ax.set_title("(b) Analytic Pipeline\nScatterer Model (Step 3 ref)")
    plt.colorbar(im, ax=ax, label="Normalized Power [dB]", shrink=0.85)

    # (c) Doppler profile comparison
    ax = fig.add_subplot(gs[0, 2])
    p_int = np.mean(10 ** (S_int / 20), axis=1)
    p_ana = np.mean(10 ** (S_ana / 20), axis=1)
    p_int_dB = 20 * np.log10(p_int + 1e-12)
    p_ana_dB = 20 * np.log10(p_ana + 1e-12)
    p_int_dB -= np.max(p_int_dB)
    p_ana_dB -= np.max(p_ana_dB)

    ax.plot(f_khz_int, p_int_dB, "b-", linewidth=1.0, alpha=0.8, label="Integrated (RT)")
    ax.plot(f_khz_ana, p_ana_dB, "r--", linewidth=1.0, alpha=0.8, label="Analytic")
    ax.axvline(x=f_body * 1e-3, color="cyan", linestyle=":", linewidth=0.8)
    ax.axvline(x=(f_body + f_dev) * 1e-3, color="orange", linestyle=":", linewidth=0.5)
    ax.axvline(x=(f_body - f_dev) * 1e-3, color="orange", linestyle=":", linewidth=0.5)
    ax.set_xlim((f_body - f_dev * 1.3) * 1e-3, (f_body + f_dev * 1.3) * 1e-3)
    ax.set_xlabel("Doppler Frequency [kHz]"); ax.set_ylabel("Normalized Power [dB]")
    ax.set_title(f"(c) Doppler Profile Comparison\nCorr = {validation['correlation']:.4f}")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # (d) Signal envelope |s(t)|
    ax = fig.add_subplot(gs[1, 0])
    env_int = np.abs(np.asarray(integrated_results["signal"]))
    env_ana = np.abs(np.asarray(analytic["signal"]))
    t_env = np.arange(len(env_int)) / cfg.sampling_rate * 1e3
    ax.plot(t_env[:500], env_int[:500], "b-", linewidth=0.6, alpha=0.7, label="Integrated")
    ax.plot(t_env[:500], env_ana[:500], "r--", linewidth=0.6, alpha=0.7, label="Analytic")
    ax.set_xlabel("Time [ms]"); ax.set_ylabel("Magnitude")
    ax.set_title("(d) Signal Envelope |s(t)|  [first 500 samples]")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # (e) RT path power distribution
    ax = fig.add_subplot(gs[1, 1])
    a_static = np.asarray(integrated_results["a_static"])
    path_powers = np.abs(a_static[0, 0, 0, 0, :, 0]) ** 2
    path_powers_sorted = np.sort(path_powers)[::-1]
    bar_colors = ["#2196F3" if i == 0 else "#90CAF9" for i in range(len(path_powers_sorted))]
    ax.bar(range(len(path_powers_sorted)), path_powers_sorted, color=bar_colors, edgecolor="white")
    ax.set_xlabel("Path Index (sorted)"); ax.set_ylabel("Power")
    ax.set_title(f"(e) RT Path Power Distribution\n{len(path_powers)} paths, total = {np.sum(path_powers):.3e}")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)

    # (f) Phase evolution of dominant path vs body scatterer
    ax = fig.add_subplot(gs[1, 2])
    a_dyn = np.asarray(integrated_results["a_dynamic"])
    a_dyn_flat = a_dyn[:, 0, 0, 0, :, 0]  # [n_t, n_paths]
    phase_dominant = np.angle(a_dyn_flat[:, 0])  # dominant path
    # Analytic body phase
    pos = modulator.scatterer_positions(np.arange(len(phase_dominant)) / cfg.sampling_rate)
    d_body = modulator._scatterer_roundtrip_distances(pos)[0]
    phase_body = -2 * np.pi * d_body / cfg.wavelength

    ax.plot(t_env[:200], phase_dominant[:200], "b-", linewidth=0.8, label="RT dominant path")
    ax.plot(t_env[:200], phase_body[:200], "r--", linewidth=0.8, label="Analytic body")
    ax.set_xlabel("Time [ms]"); ax.set_ylabel("Phase [rad]")
    ax.set_title("(f) Phase Evolution  [first 200 samples]")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    fig_path = OUT_DIR / f"integration_{output_name}.png"
    fig.savefig(fig_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  Figure saved: {fig_path}")


def plot_summary_figure(
    all_integrated: list[dict],
    analytic: dict,
    all_validation: list[dict],
    modulator: MicroDopplerModulator,
    benchmarks: dict,
):
    """Generate a summary figure across all three scenes."""
    fig = plt.figure(figsize=(16, 12))
    cfg = modulator.cfg
    f_body = analytic["body_doppler"]

    fig.suptitle(
        f"Step 5 — Integration Validation Summary\n"
        f"fc={cfg.carrier_freq/1e9:.0f} GHz  β={modulator.beta:.0f}  "
        f"f_rot={cfg.rotation_freq} Hz  v_body={cfg.body_velocity[0]:.0f} m/s",
        fontsize=11, fontweight="bold",
    )

    gs = GridSpec(2, 3, figure=fig, hspace=0.50, wspace=0.38)

    vmin = -40

    # ── Row 1: Spectrograms for each scene ──────────────────────────────────
    scene_keys_short = ["paris_los", "paris_multipath", "floor_wall"]
    for i, (integrated, val) in enumerate(zip(all_integrated, all_validation)):
        ax = fig.add_subplot(gs[0, i])
        f_sc = np.atleast_1d(np.asarray(integrated["f_stft"], dtype=np.float64))
        t_sc = np.atleast_1d(np.asarray(integrated["t_stft"], dtype=np.float64))
        S_sc = np.atleast_2d(np.asarray(integrated["S_dB"], dtype=np.float64))
        im = ax.pcolormesh(t_sc * 1e3, f_sc * 1e-3, S_sc,
                           shading="gouraud", vmin=vmin, vmax=0, cmap="inferno")
        ax.axhline(y=f_body * 1e-3, color="cyan", linestyle="--", linewidth=0.8)
        ax.set_xlabel("Time [ms]"); ax.set_ylabel("Doppler [kHz]")
        ax.set_title(f"{integrated['label']}\n{integrated['n_paths']} paths, "
                     f"corr={val['correlation']:.3f}", fontsize=9)
        plt.colorbar(im, ax=ax, label="dB", shrink=0.85)

    # ── Row 2: Comparisons ──────────────────────────────────────────────────
    # (d) Doppler profiles overlay
    ax = fig.add_subplot(gs[1, 0])
    colors = ["#2196F3", "#4CAF50", "#FF9800"]
    for integrated, color, key in zip(all_integrated, colors, scene_keys_short):
        S_arr = np.atleast_2d(np.asarray(integrated["S_dB"], dtype=np.float64))
        f_arr = np.atleast_1d(np.asarray(integrated["f_stft"], dtype=np.float64))
        p = np.mean(10 ** (S_arr / 20), axis=1)
        p_dB = 20 * np.log10(p + 1e-12)
        p_dB -= np.max(p_dB)
        ax.plot(f_arr * 1e-3, p_dB, color=color, linewidth=0.8, alpha=0.8, label=key)

    # Add analytic reference
    S_ana_ref = np.atleast_2d(np.asarray(analytic["S_dB"], dtype=np.float64))
    f_ana_ref = np.atleast_1d(np.asarray(analytic["f_stft"], dtype=np.float64))
    p_ana = np.mean(10 ** (S_ana_ref / 20), axis=1)
    p_ana_dB = 20 * np.log10(p_ana + 1e-12)
    p_ana_dB -= np.max(p_ana_dB)
    ax.plot(f_ana_ref * 1e-3, p_ana_dB, "k--", linewidth=1.0, alpha=0.6, label="analytic ref")

    ax.set_xlim((f_body - modulator.f_dev_peak * 1.3) * 1e-3,
                (f_body + modulator.f_dev_peak * 1.3) * 1e-3)
    ax.set_xlabel("Doppler Frequency [kHz]"); ax.set_ylabel("Normalized Power [dB]")
    ax.set_title("(d) Doppler Profiles — All Scenes vs Analytic")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # (e) Validation metrics bar chart
    ax = fig.add_subplot(gs[1, 1])
    x = np.arange(len(scene_keys_short))
    width = 0.25
    corr_vals = [v["correlation"] for v in all_validation]
    spread_int_vals = [v["spread_int"] * 100 for v in all_validation]
    spread_ana_vals = [v["spread_ana"] * 100 for v in all_validation]

    ax.bar(x - width, corr_vals, width, color="#2196F3", label="Correlation")
    ax.bar(x, spread_int_vals, width, color="#4CAF50", label="Spread (integrated) %")
    ax.bar(x + width, spread_ana_vals, width, color="#FF9800", label="Spread (analytic) %")
    ax.set_xticks(x); ax.set_xticklabels(scene_keys_short, fontsize=8)
    ax.set_ylabel("Value")
    ax.set_title("(e) Validation Metrics")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3, axis="y")

    # (f) Performance benchmarks
    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    lines = [
        "=" * 50,
        "  PERFORMANCE BENCHMARKS",
        "=" * 50,
        "",
        f"  RT solve (per scene):",
    ]
    for bm in benchmarks["per_scene"]:
        lines.append(f"    {bm['key']:<20s}  {bm['t_rt']*1e3:>6.1f} ms  "
                     f"({bm['n_paths']} paths)")

    lines += [
        "",
        f"  modulate_cir() avg:   {benchmarks['t_mod_avg']*1e3:.1f} ms",
        f"  modulate_cir() max:   {benchmarks['t_mod_max']*1e3:.1f} ms",
        f"  STFT avg:             {benchmarks['t_stft_avg']*1e3:.1f} ms",
        "",
        f"  Total pipeline avg:   {benchmarks['t_total_avg']*1e3:.0f} ms",
        f"  Total pipeline max:   {benchmarks['t_total_max']*1e3:.0f} ms",
        "",
        f"  n_samples:            {benchmarks['n_samples']}",
        f"  Sample rate:          {cfg.sampling_rate*1e-3:.0f} kHz",
        f"  Obs duration:         {cfg.obs_duration*1e3:.0f} ms",
    ]

    # Check targets
    rt_ok = all(b["t_rt"] < 1.0 for b in benchmarks["per_scene"])
    mod_ok = benchmarks["t_mod_max"] < 0.1
    total_ok = benchmarks["t_total_max"] < 2.0

    lines += [
        "",
        "  TARGETS:",
        f"    RT < 1s:             {'✓ PASS' if rt_ok else '✗ FAIL'}",
        f"    modulate_cir < 0.1s: {'✓ PASS' if mod_ok else '✗ FAIL'}",
        f"    Total < 2s:          {'✓ PASS' if total_ok else '✗ FAIL'}",
        "",
        f"  ALL TARGETS: {'✓ PASS' if (rt_ok and mod_ok and total_ok) else '✗ FAIL'}",
    ]

    ax.text(0, 0.95, "\n".join(lines), transform=ax.transAxes,
            fontfamily="monospace", fontsize=8, va="top")

    fig_path = OUT_DIR / "integration_summary.png"
    fig.savefig(fig_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Summary figure saved: {fig_path}")


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main(scenes_to_run: list[str] | None = None, skip_plots: bool = False):
    """Run the full integration validation.

    Parameters
    ----------
    scenes_to_run : list[str] or None
        Subset of scene keys to run. None → all three.
    skip_plots : bool
        If True, skip figure generation.
    """
    if scenes_to_run is None:
        scenes_to_run = ["paris_los", "paris_multipath", "floor_wall"]

    print("=" * 64)
    print("  Step 5 — Micro-Doppler Integration Demo")
    print("  Sionna RT CIR Pipeline + MicroDopplerModulator")
    print("=" * 64)

    # ── Setup modulator ─────────────────────────────────────────────────────
    cfg = UAVMicroDopplerConfig(
        carrier_freq=MODULATOR_CFG["carrier_freq"],
        body_position=(scenes_to_run and
                       SCENE_CONFIGS[scenes_to_run[0]]["body_pos"] or
                       [0.0, 0.0, 10.0]),
        radar_position=(scenes_to_run and
                        SCENE_CONFIGS[scenes_to_run[0]]["radar_pos"] or
                        [-40.0, 0.0, 10.0]),
        body_velocity=MODULATOR_CFG["body_velocity"],
        sampling_rate=MODULATOR_CFG["sampling_rate"],
        obs_duration=MODULATOR_CFG["obs_duration"],
        snr_db=MODULATOR_CFG["snr_db"],
    )
    modulator = MicroDopplerModulator(cfg)
    print(modulator.summary())

    t_vec = np.arange(int(cfg.sampling_rate * cfg.obs_duration)) / cfg.sampling_rate
    print(f"\n  Time vector: {len(t_vec)} samples @ {cfg.sampling_rate*1e-3:.0f} kHz "
          f"({cfg.obs_duration*1e3:.0f} ms)")

    # ── Run analytic pipeline (once, shared reference) ──────────────────────
    analytic = run_analytic_pipeline(modulator, t_vec)

    # ── Run integrated pipeline for each scene ──────────────────────────────
    all_integrated = []
    all_validation = []

    for scene_key in scenes_to_run:
        scfg = SCENE_CONFIGS[scene_key]
        # Update modulator config for this scene's geometry
        scene_cfg = UAVMicroDopplerConfig(
            carrier_freq=MODULATOR_CFG["carrier_freq"],
            body_position=scfg["body_pos"],
            radar_position=scfg["radar_pos"],
            body_velocity=MODULATOR_CFG["body_velocity"],
            sampling_rate=MODULATOR_CFG["sampling_rate"],
            obs_duration=MODULATOR_CFG["obs_duration"],
            snr_db=MODULATOR_CFG["snr_db"],
        )
        scene_modulator = MicroDopplerModulator(scene_cfg)
        t_scene = np.arange(int(scene_cfg.sampling_rate * scene_cfg.obs_duration)) / scene_cfg.sampling_rate

        integrated = run_integrated_pipeline(scene_key, scene_modulator, t_scene)
        validation = validate_spectrograms(integrated, analytic, scene_key)
        all_integrated.append(integrated)
        all_validation.append(validation)

        if not skip_plots:
            plot_comparison_figure(integrated, analytic, validation,
                                   scene_modulator, scene_key)

    # ── Performance benchmarks ──────────────────────────────────────────────
    t_rt_all = [r["t_rt"] for r in all_integrated]
    t_mod_all = [r["t_mod"] for r in all_integrated]
    t_stft_all = [r["t_stft_elapsed"] for r in all_integrated]
    t_total_all = [r["t_total"] for r in all_integrated]

    benchmarks = {
        "per_scene": [
            {
                "key": r["scene_key"],
                "t_rt": r["t_rt"],
                "n_paths": r["n_paths"],
            }
            for r in all_integrated
        ],
        "t_mod_avg": np.mean(t_mod_all),
        "t_mod_max": np.max(t_mod_all),
        "t_stft_avg": np.mean(t_stft_all),
        "t_total_avg": np.mean(t_total_all),
        "t_total_max": np.max(t_total_all),
        "n_samples": len(t_vec),
    }

    # ── Print final summary ─────────────────────────────────────────────────
    print()
    print("=" * 64)
    print("  Step 5 — Final Validation Report")
    print("=" * 64)

    rt_ok = all(t < 1.0 for t in t_rt_all)
    mod_ok = benchmarks["t_mod_max"] < 0.1
    total_ok = benchmarks["t_total_max"] < 2.0

    # Pre-compute formatted values to avoid f-string backslash issues (Py<3.12)
    rt_max_ms = max(t_rt_all) * 1e3
    mod_max_ms = benchmarks["t_mod_max"] * 1e3
    total_max_ms = benchmarks["t_total_max"] * 1e3
    check_mark = "✓"   # ✓
    cross_mark = "✗"   # ✗

    print(f"\n  Performance Benchmarks:")
    print(f"  {'Metric':<30s} {'Target':>10s} {'Measured':>15s} {'Status':>10s}")
    print(f"  {'─'*65}")
    status_rt = f"{check_mark}" if rt_ok else f"{cross_mark}"
    print(f"  {'RT solve':<30s} {'< 1s':>10s} {f'{rt_max_ms:.0f} ms':>15s} {status_rt:>10s}")
    status_mod = f"{check_mark}" if mod_ok else f"{cross_mark}"
    print(f"  {'modulate_cir()':<30s} {'< 0.1s':>10s} {f'{mod_max_ms:.1f} ms':>15s} {status_mod:>10s}")
    status_tot = f"{check_mark}" if total_ok else f"{cross_mark}"
    print(f"  {'Total pipeline':<30s} {'< 2s':>10s} {f'{total_max_ms:.0f} ms':>15s} {status_tot:>10s}")

    print(f"\n  Spectrogram Validation:")
    print(f"  {'Scene':<25s} {'Correlation':>12s} {'Spread (int)':>14s} "
          f"{'Spread (ana)':>14s} {'Paths':>8s}")
    print(f"  {'─'*75}")
    for integrated, val in zip(all_integrated, all_validation):
        print(f"  {integrated['scene_key']:<25s} {val['correlation']:>12.4f} "
              f"{val['spread_int']*100:>13.1f}% {val['spread_ana']*100:>13.1f}% "
              f"{integrated['n_paths']:>8d}")

    all_pass = rt_ok and mod_ok and total_ok
    # Correlation should be non-negative (same physical model)
    corr_ok = all(v["correlation"] > -0.1 for v in all_validation)
    # Spectral spread should be non-zero (micro-Doppler is present)
    spread_ok = all(v["spread_int"] > 0.01 for v in all_validation)

    targets_str = f"{check_mark} ALL PASS" if all_pass else f"{cross_mark} SOME FAIL"
    corr_str = f"{check_mark}" if corr_ok else f"{cross_mark}"
    spread_str = f"{check_mark}" if spread_ok else f"{cross_mark}"
    print(f"\n  Targets:  {targets_str}")
    print(f"  Spectrogram correlation: {corr_str}")
    print(f"  Micro-Doppler spreading: {spread_str}")

    if not skip_plots:
        plot_summary_figure(all_integrated, analytic, all_validation,
                            modulator, benchmarks)

    print()
    print("=" * 64)
    print("  Done — Step 5 complete.")
    print("=" * 64)

    return {
        "integrated": all_integrated,
        "analytic": analytic,
        "validation": all_validation,
        "benchmarks": benchmarks,
        "all_pass": all_pass and corr_ok and spread_ok,
    }


# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Step 5: Micro-Doppler Integration Demo"
    )
    parser.add_argument("--scenes", nargs="+",
                        choices=["paris_los", "paris_multipath", "floor_wall"],
                        default=None,
                        help="Scenes to run (default: all three)")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip figure generation")
    args = parser.parse_args()

    result = main(scenes_to_run=args.scenes, skip_plots=args.no_plots)
    sys.exit(0 if result["all_pass"] else 1)
