#!/usr/bin/env python3
"""Baseline Doppler Validation — Sionna RT with Paris (etoile) scene.

UE moves along x-axis at constant velocity v=10 m/s.
100 CIR snapshots collected at Δt=0.5 ms intervals.
Doppler shift extracted from phase progression of the dominant (LOS) path
and compared against the theoretical prediction and path-solver built-in value.

Expected: direct-path Doppler ≈ 933 Hz at 28 GHz carrier.
"""

from __future__ import annotations

import time
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.constants import c as SPEED_OF_LIGHT

import sionna
from sionna.rt import (
    load_scene,
    PathSolver,
    PlanarArray,
    Receiver,
    Transmitter,
)

# ── Output directory ────────────────────────────────────────────────────────
OUT_DIR = Path(__file__).resolve().parent.parent / "figures"
OUT_DIR.mkdir(exist_ok=True)

# ══════════════════════════════════════════════════════════════════════════════
# 1. Parameters
# ══════════════════════════════════════════════════════════════════════════════

FC = 28e9             # Carrier frequency [Hz]
V_UE = 10.0           # UE velocity along +x axis [m/s]
DT = 0.5e-3           # Snapshot sampling interval [s]
N_SNAPSHOTS = 100     # Number of CIR snapshots
FS = 122.88e6         # CIR sampling frequency [Hz]

# BS fixed position (TX) — placed at same height as UE for dominant x-axis LOS
BS_POS = np.array([-40.0, 0.0, 10.0], dtype=np.float32)

# UE start position (RX)
UE_START = np.array([0.0, 0.0, 10.0], dtype=np.float32)

# Path solver settings
MAX_DEPTH = 5
LOS = True
SPECULAR = True
DIFFUSE = True
DIFFRACTION = False
EDGE_DIFFRACTION = False
REFRACTION = False

# ══════════════════════════════════════════════════════════════════════════════
# 2. Derived quantities
# ══════════════════════════════════════════════════════════════════════════════

WAVELENGTH = SPEED_OF_LIGHT / FC                    # ≈ 10.7 mm
MAX_DOPPLER = FC * V_UE / SPEED_OF_LIGHT            # ≈ 933.3 Hz
STEP_DISTANCE = V_UE * DT                           # 0.005 m
TOTAL_TIME = DT * (N_SNAPSHOTS - 1)                 # 0.0495 s
TOTAL_DISTANCE = V_UE * TOTAL_TIME                  # 0.495 m
PHASE_PER_STEP = 2 * np.pi * MAX_DOPPLER * DT       # ~2.93 rad — well within Nyquist

# Theoretical Doppler for LOS
# f_d = fc * v_radial / c, where v_radial > 0 means approaching (distance decreasing)
# v_radial = v_ue · unit(TX - RX) = projection of UE velocity toward TX
dx0 = UE_START[0] - BS_POS[0]
dy0 = UE_START[1] - BS_POS[1]
dz0 = UE_START[2] - BS_POS[2]
d0 = np.sqrt(dx0**2 + dy0**2 + dz0**2)
unit_tx_to_rx_x = dx0 / d0                         # x-comp of unit vector TX → RX
v_radial = V_UE * (-unit_tx_to_rx_x)               # UE velocity toward TX (>0 = approaching)
F_DOPPLER_THEO = FC * v_radial / SPEED_OF_LIGHT    # Theoretical Doppler [Hz]

# Check Nyquist: |f_d_max| < 1/(2*DT) = 1000 Hz
NYQUIST_LIMIT = 1.0 / (2 * DT)
assert abs(F_DOPPLER_THEO) < NYQUIST_LIMIT, (
    f"Doppler {F_DOPPLER_THEO:.0f} Hz exceeds Nyquist limit {NYQUIST_LIMIT:.0f} Hz!"
)

print("=" * 64)
print("  Baseline Doppler Validation — Paris (etoile)")
print("=" * 64)
print(f"  fc          = {FC / 1e9:.1f} GHz")
print(f"  λ           = {WAVELENGTH * 1e3:.2f} mm")
print(f"  v_UE        = {V_UE} m/s  (along +x)")
print(f"  Δt          = {DT * 1e3:.2f} ms")
print(f"  N_snapshots = {N_SNAPSHOTS}")
print(f"  FS (CIR)    = {FS / 1e6:.1f} MHz")
print(f"  Nyquist     = ±{NYQUIST_LIMIT:.0f} Hz")
print(f"  f_d (max)   = {MAX_DOPPLER:.1f} Hz  (v_radial = V_UE)")
print(f"  f_d (theo)  = {F_DOPPLER_THEO:.1f} Hz  (LOS projection)")
print()

# ══════════════════════════════════════════════════════════════════════════════
# 3. Scene setup
# ══════════════════════════════════════════════════════════════════════════════

print("Loading scene ...", end=" ", flush=True)
t0 = time.time()
scene = load_scene(sionna.rt.scene.etoile)
scene.frequency = FC
print(f"done ({time.time() - t0:.1f}s)")

# Single-element isotropic antennas
scene.tx_array = PlanarArray(
    num_rows=1, num_cols=1,
    vertical_spacing=0.5, horizontal_spacing=0.5,
    pattern="iso", polarization="V",
)
scene.rx_array = PlanarArray(
    num_rows=1, num_cols=1,
    vertical_spacing=0.5, horizontal_spacing=0.5,
    pattern="iso", polarization="V",
)

# Place devices — BS as TX, UE as RX (following our convention: BS=TX, UE=RX)
scene.add(Transmitter(
    name="bs",
    position=BS_POS,
    orientation=[0, 0, 0],
    power_dbm=0.0,
))
scene.add(Receiver(
    name="ue",
    position=UE_START,
    orientation=[0, 0, 0],
))

print(f"  BS (TX) @ {BS_POS}")
print(f"  UE (RX) @ {UE_START}  (start)")
print(f"  LOS distance = {d0:.1f} m")
print()

# ══════════════════════════════════════════════════════════════════════════════
# 4. Doppler measurement loop
# ══════════════════════════════════════════════════════════════════════════════

solver = PathSolver()

phases_los = []          # Phase of dominant path at each snapshot
powers_los = []          # Power of dominant path
dopplers_solver = []     # Built-in path-solver Doppler for dominant path
num_paths_per_step = []  # Number of paths found
ue_positions = []        # UE position history

print(f"Running {N_SNAPSHOTS} snapshots ...")
t_loop = time.time()

for step in range(N_SNAPSHOTS):
    # Update UE position
    ue_pos = np.array([
        UE_START[0] + V_UE * step * DT,
        UE_START[1],
        UE_START[2],
    ], dtype=np.float32)
    scene.get("ue").position = ue_pos
    ue_positions.append(ue_pos.copy())

    # Ray tracing
    paths = solver(
        scene,
        max_depth=MAX_DEPTH,
        los=LOS,
        specular_reflection=SPECULAR,
        diffuse_reflection=DIFFUSE,
        diffraction=DIFFRACTION,
        edge_diffraction=EDGE_DIFFRACTION,
        refraction=REFRACTION,
        synthetic_array=False,
    )

    # CIR: shape [num_rx, num_time_steps, num_tx, num_tx_ants, num_paths, num_rx_ants]
    a, _ = paths.cir(sampling_frequency=FS, normalize_delays=False, out_type="numpy")
    a0 = np.asarray(a[0, :, 0, 0, :, 0], dtype=np.complex64)  # [time_steps, paths]

    # Dominant path (LOS) selection by power
    path_power = np.sum(np.abs(a0) ** 2, axis=0)          # [num_paths]
    best_idx = int(np.argmax(path_power))

    phases_los.append(np.angle(a0[0, best_idx]))
    powers_los.append(path_power[best_idx])

    # Path-solver built-in Doppler
    doppler_all = np.asarray(paths.doppler.numpy(), dtype=np.float32)  # [num_rx, num_paths]
    dopplers_solver.append(float(np.squeeze(doppler_all)[best_idx]))

    num_paths_per_step.append(path_power.shape[0])

    if (step + 1) % 25 == 0:
        elapsed = time.time() - t_loop
        rate = (step + 1) / elapsed
        eta = (N_SNAPSHOTS - step - 1) / rate
        print(f"  [{step+1:3d}/{N_SNAPSHOTS}]  "
              f"UE x={ue_pos[0]:.3f} m  "
              f"paths={path_power.shape[0]}  "
              f"|  {rate:.1f} snap/s  ETA {eta:.0f}s")

elapsed = time.time() - t_loop
print(f"  Done — {elapsed:.0f}s total, {N_SNAPSHOTS/elapsed:.1f} snap/s avg")
print()

# Convert to arrays
phases_los = np.array(phases_los, dtype=np.float64)
powers_los = np.array(powers_los, dtype=np.float64)
dopplers_solver = np.array(dopplers_solver, dtype=np.float64)
num_paths_per_step = np.array(num_paths_per_step, dtype=np.int32)
ue_positions = np.array(ue_positions, dtype=np.float64)
t_snap = np.arange(N_SNAPSHOTS) * DT

# ══════════════════════════════════════════════════════════════════════════════
# 5. Doppler extraction from phase
# ══════════════════════════════════════════════════════════════════════════════

# Unwrap phase (handles 2π jumps)
phases_unwrapped = np.unwrap(phases_los)

# Linear regression:  φ(t) = 2π · f_d · t + φ₀
A = np.stack([t_snap, np.ones_like(t_snap)], axis=1)
slope, intercept = np.linalg.lstsq(A, phases_unwrapped, rcond=None)[0]
f_doppler_measured = slope / (2 * np.pi)
phase_fit = slope * t_snap + intercept
phase_residual = phases_unwrapped - phase_fit

# RMS phase residual
rms_residual_rad = np.std(phase_residual)
rms_residual_deg = np.rad2deg(rms_residual_rad)

# ══════════════════════════════════════════════════════════════════════════════
# 6. Results summary
# ══════════════════════════════════════════════════════════════════════════════

f_err_abs = abs(f_doppler_measured - F_DOPPLER_THEO)
f_err_rel = f_err_abs / max(abs(F_DOPPLER_THEO), 1.0)

# Path-solver Doppler stats
doppler_solver_mean = np.mean(dopplers_solver)
doppler_solver_std = np.std(dopplers_solver)

# Power stability
power_drift_db = 10 * np.log10(powers_los / powers_los[0])

print("=" * 64)
print("  Results")
print("=" * 64)
print(f"  Doppler (from phase)      : {f_doppler_measured:+.2f} Hz")
print(f"  Doppler (theoretical LOS) : {F_DOPPLER_THEO:+.2f} Hz")
print(f"  Doppler (path solver)     : {doppler_solver_mean:+.2f} Hz  (±{doppler_solver_std:.2f})")
print(f"  Absolute error            : {f_err_abs:.2f} Hz")
print(f"  Relative error            : {f_err_rel*100:.2f} %")
print(f"  RMS phase residual        : {rms_residual_deg:.2f}°  ({rms_residual_rad:.4f} rad)")
print(f"  Path count (mean)         : {np.mean(num_paths_per_step):.1f}  (min={np.min(num_paths_per_step)}, max={np.max(num_paths_per_step)})")
print(f"  Power variation           : {np.max(power_drift_db)-np.min(power_drift_db):.2f} dB pk-pk")
print()

# Validation verdict
if f_err_rel < 0.05:
    print("  ✓ PASS — Measured Doppler matches theory (< 5% error)")
elif f_err_rel < 0.10:
    print("  ~ MARGINAL — Doppler error between 5-10%")
else:
    print("  ✗ FAIL — Doppler error > 10%, check setup")

# ══════════════════════════════════════════════════════════════════════════════
# 7. Visualization
# ══════════════════════════════════════════════════════════════════════════════

plt.rcParams.update({
    "figure.dpi": 150,
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
})

fig, axes = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle(
    f"Baseline Doppler Validation — Paris (etoile)\n"
    f"fc={FC/1e9:.0f}GHz  v={V_UE}m/s  Δt={DT*1e3:.1f}ms  N={N_SNAPSHOTS}",
    fontsize=11, fontweight="bold",
)

# ── (a) Unwrapped Phase vs Time ─────────────────────────────────────────────
ax = axes[0, 0]
ax.plot(t_snap * 1e3, phases_unwrapped, "b.-", markersize=3, linewidth=0.8,
        label="Measured (unwrapped)")
ax.plot(t_snap * 1e3, phase_fit, "r--", linewidth=1.2,
        label=f"Linear fit: f_d={f_doppler_measured:+.1f} Hz")
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Phase [rad]")
ax.set_title("(a) Unwrapped Phase vs Time")
ax.legend(loc="best")
ax.grid(True, alpha=0.3)

# Annotate slope
mid_t = t_snap[N_SNAPSHOTS // 2] * 1e3
mid_phi = phase_fit[N_SNAPSHOTS // 2]
ax.annotate(
    f"slope = 2π·{f_doppler_measured:.1f} rad/s",
    xy=(mid_t, mid_phi),
    xytext=(mid_t + 5, mid_phi - 2),
    fontsize=8, color="red",
    arrowprops=dict(arrowstyle="->", color="red", lw=0.8),
)

# ── (b) Phase Residual ──────────────────────────────────────────────────────
ax = axes[0, 1]
ax.plot(t_snap * 1e3, phase_residual * 180 / np.pi, "k.-", markersize=3, linewidth=0.8)
ax.axhline(0, color="gray", linestyle="--", linewidth=0.5)
ax.axhline(+rms_residual_deg, color="red", linestyle=":", linewidth=0.8,
           label=f"±RMS = {rms_residual_deg:.2f}°")
ax.axhline(-rms_residual_deg, color="red", linestyle=":", linewidth=0.8)
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Phase Residual [deg]")
ax.set_title("(b) Phase Residual after Linear Fit")
ax.legend(loc="best")
ax.grid(True, alpha=0.3)

# ── (c) Doppler Comparison ───────────────────────────────────────────────────
ax = axes[1, 0]
methods = ["Phase Fit", "Path Solver", "Theory (LOS)"]
values = [f_doppler_measured, doppler_solver_mean, F_DOPPLER_THEO]
errors = [0, doppler_solver_std, 0]
colors = ["#2196F3", "#4CAF50", "#FF9800"]
bars = ax.bar(methods, values, yerr=errors, color=colors, capsize=6, width=0.5)
ax.axhline(y=0, color="gray", linewidth=0.5)
ax.set_ylabel("Doppler Shift [Hz]")
ax.set_title("(c) Doppler Comparison")
ax.grid(True, alpha=0.3, axis="y")

# Value labels on bars
for bar, val in zip(bars, values):
    ax.text(bar.get_x() + bar.get_width() / 2, val + np.sign(val) * 15,
            f"{val:+.1f} Hz", ha="center", fontsize=8, fontweight="bold")

# ── (d) Path Power Stability ───────────────────────────────────────────────
ax = axes[1, 1]
ax.plot(t_snap * 1e3, power_drift_db, "g.-", markersize=3, linewidth=0.8)
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Relative Power [dB]")
ax.set_title("(d) Dominant Path Power Stability")
ax.grid(True, alpha=0.3)

mean_pwr = np.mean(power_drift_db)
ax.axhline(mean_pwr, color="red", linestyle="--", linewidth=0.8,
           label=f"Mean = {mean_pwr:.2f} dB")
ax.legend(loc="best")

plt.tight_layout()

# Save
fig_path = OUT_DIR / "baseline_doppler_etoile.png"
fig.savefig(fig_path, dpi=200, bbox_inches="tight")
print(f"Figure saved to {fig_path}")

# Also save phase-only diagnostic plot
fig2, ax2 = plt.subplots(figsize=(8, 3))
ax2.plot(t_snap * 1e3, phases_los * 180 / np.pi, "b.-", markersize=3, linewidth=0.8,
         label="Raw phase (wrapped)")
ax2.set_xlabel("Time [ms]")
ax2.set_ylabel("Phase [deg]")
ax2.set_title("Raw Wrapped Phase (sanity check — should be linear with 180° wraps)")
ax2.legend(loc="best")
ax2.grid(True, alpha=0.3)
fig2.tight_layout()
fig2_path = OUT_DIR / "baseline_doppler_wrapped_phase.png"
fig2.savefig(fig2_path, dpi=200, bbox_inches="tight")
print(f"Figure saved to {fig2_path}")

plt.show()
print("\nDone.")
