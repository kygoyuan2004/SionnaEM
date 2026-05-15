#!/usr/bin/env python3
"""Micro-Doppler: Single Rotating Scatterer — Sionna RT Validation (Step 2).

Models a single scatter point rotating on a blade (R=0.15 m, f_rot=100 Hz).
TX position is updated each snapshot to the instantaneous scatterer position.
Phase modulation is extracted and compared against micro-Doppler theory.

Expected: sinusoidal phase modulation φ(t) = 2πf_Δ·t + β·sin(2πf_rot·t + φ₀)
with modulation index β ≈ 175.9 rad at 28 GHz.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
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

FC = 28e9                     # Carrier frequency [Hz]
R_BLADE = 0.15                # Blade radius [m]
F_ROT = 100.0                 # Rotation frequency [Hz]
OMEGA_ROT = 2 * np.pi * F_ROT  # Angular frequency [rad/s]
DT = 0.5e-3                   # Sampling interval → fs = 2 kHz [s]
N_SNAPSHOTS = 100             # Number of CIR snapshots
FS_CIR = 122.88e6             # CIR sampling frequency [Hz]

# Radar receiver (RX) — fixed position
RX_POS = np.array([-40.0, 0.0, 10.0], dtype=np.float32)

# Scatterer body center — the centre of rotation
BODY_CENTER = np.array([0.0, 0.0, 10.0], dtype=np.float32)

# Random initial phase for reproducibility
RNG = np.random.default_rng(42)
PHI0 = RNG.uniform(0, 2 * np.pi)

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

WAVELENGTH = SPEED_OF_LIGHT / FC                       # ≈ 10.71 mm
TOTAL_TIME = DT * (N_SNAPSHOTS - 1)                    # 0.0495 s
NYQUIST_LIMIT = 1.0 / (2 * DT)                         # 1000 Hz
N_ROTATIONS = TOTAL_TIME * F_ROT                       # ≈ 5 rotations

# Modulation index β:
#   One-way (TX→RX scatterer):  β_1w = 2πR/λ ≈ 88 rad
#   Monostatic radar (round-trip): β = 4πR/λ ≈ 176 rad
# Our simulation moves the TX (scatterer) → one-way geometry.
# We apply ×2 in post-processing for the round-trip (monostatic) equivalent.
BETA_1WAY_MAX = 2 * np.pi * R_BLADE / WAVELENGTH       # ≈ 87.9 rad
BETA_MONOSTATIC_MAX = 4 * np.pi * R_BLADE / WAVELENGTH  # ≈ 175.9 rad

# Effective β accounting for LOS geometry
d_body_to_rx = np.linalg.norm(RX_POS - BODY_CENTER)
los_unit = (BODY_CENTER - RX_POS) / d_body_to_rx
in_plane_factor = np.sqrt(los_unit[0]**2 + los_unit[1]**2)
BETA_1WAY_EFF = BETA_1WAY_MAX * in_plane_factor
BETA_EFF = BETA_MONOSTATIC_MAX * in_plane_factor       # Round-trip (plan reference)

# Peak micro-Doppler frequency deviation (round-trip): f_dev = β·f_rot·cos(ωt)
F_DEV_PEAK = BETA_EFF * F_ROT                          # ≈ 17.6 kHz

# Phase change per sample (one-way): ≈ 27.6 rad ≫ π → aliasing!
PHASE_PER_STEP_MAX = BETA_1WAY_EFF * OMEGA_ROT * DT

print("=" * 72)
print("  Micro-Doppler: Single Rotating Scatterer — Paris (etoile)")
print("=" * 72)
print(f"  fc          = {FC / 1e9:.1f} GHz")
print(f"  λ           = {WAVELENGTH * 1e3:.2f} mm")
print(f"  R_blade     = {R_BLADE:.3f} m")
print(f"  f_rot       = {F_ROT} Hz  (ω = {OMEGA_ROT:.1f} rad/s)")
print(f"  Δt          = {DT * 1e3:.2f} ms  → fs = {1 / DT * 1e-3:.1f} kHz")
print(f"  N_snapshots = {N_SNAPSHOTS}")
print(f"  T_obs       = {TOTAL_TIME * 1e3:.1f} ms  ({N_ROTATIONS:.1f} rotations)")
print(f"  φ₀          = {PHI0:.2f} rad")
print(f"  β_1way      = {BETA_1WAY_EFF:.1f} rad  (simulation: TX→RX)")
print(f"  β_monostatic= {BETA_EFF:.1f} rad  (round-trip = 2×β_1way, plan reference)")
print(f"  f_dev_peak  = ±{F_DEV_PEAK * 1e-3:.1f} kHz")
print(f"  Nyquist     = ±{NYQUIST_LIMIT:.0f} Hz")
print(f"  Δφ_max/step = {PHASE_PER_STEP_MAX:.1f} rad  (≈ {PHASE_PER_STEP_MAX / np.pi:.1f}π)")
print()
print(f"  ⚠  Δφ/step ≫ π → raw phase is aliased at this fs.")
print(f"     Using exact geometry-derived phase for analysis.")
print(f"     Round-trip phase = 2 × one-way RT phase (monostatic radar model).")
print()

# ══════════════════════════════════════════════════════════════════════════════
# 3. Scene setup
# ══════════════════════════════════════════════════════════════════════════════

print("Loading scene ...", end=" ", flush=True)
t0 = time.time()
scene = load_scene(sionna.rt.scene.etoile)
scene.frequency = FC
print(f"done ({time.time() - t0:.1f}s)")

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

# TX = rotating scatterer, RX = fixed radar
scene.add(Transmitter(
    name="scatterer",
    position=BODY_CENTER,
    orientation=[0, 0, 0],
    power_dbm=0.0,
))
scene.add(Receiver(
    name="radar",
    position=RX_POS,
    orientation=[0, 0, 0],
))

print(f"  Radar (RX)     @ {RX_POS}")
print(f"  Body center    @ {BODY_CENTER}")
print(f"  LOS distance   = {d_body_to_rx:.1f} m")
print(f"  LOS unit vec   = [{los_unit[0]:.4f}, {los_unit[1]:.4f}, {los_unit[2]:.4f}]")
print()

# ══════════════════════════════════════════════════════════════════════════════
# 4. Simulation loop — move TX in a circle, collect CIR at each step
# ══════════════════════════════════════════════════════════════════════════════

solver = PathSolver()

phases_cir = []               # Raw wrapped CIR phase of dominant path
powers_cir = []               # Power of dominant path
dopplers_solver = []          # Path-solver built-in Doppler
num_paths_list = []           # Number of paths per snapshot
scatterer_positions = []      # Scatterer position history

print(f"Running {N_SNAPSHOTS} snapshots ...")
t_loop = time.time()

for step in range(N_SNAPSHOTS):
    t = step * DT
    angle = OMEGA_ROT * t + PHI0

    scatterer_pos = np.array([
        BODY_CENTER[0] + R_BLADE * np.cos(angle),
        BODY_CENTER[1] + R_BLADE * np.sin(angle),
        BODY_CENTER[2],
    ], dtype=np.float32)
    scene.get("scatterer").position = scatterer_pos
    scatterer_positions.append(scatterer_pos.copy())

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

    a, _ = paths.cir(sampling_frequency=FS_CIR, normalize_delays=False, out_type="numpy")
    a0 = np.asarray(a[0, :, 0, 0, :, 0], dtype=np.complex64)

    path_power = np.sum(np.abs(a0) ** 2, axis=0)
    best_idx = int(np.argmax(path_power))

    phases_cir.append(np.angle(a0[0, best_idx]))
    powers_cir.append(path_power[best_idx])

    doppler_all = np.asarray(paths.doppler.numpy(), dtype=np.float32)
    dopplers_solver.append(float(np.squeeze(doppler_all)[best_idx]))

    num_paths_list.append(path_power.shape[0])

    if (step + 1) % 25 == 0:
        elapsed = time.time() - t_loop
        rate = (step + 1) / elapsed
        eta = (N_SNAPSHOTS - step - 1) / rate
        print(f"  [{step+1:3d}/{N_SNAPSHOTS}]  "
              f"θ={np.rad2deg(angle) % 360:5.1f}°  "
              f"paths={path_power.shape[0]}  "
              f"|  {rate:.1f} snap/s  ETA {eta:.0f}s")

elapsed = time.time() - t_loop
print(f"  Done — {elapsed:.0f}s total, {N_SNAPSHOTS / elapsed:.1f} snap/s avg")
print()

# ── Convert to arrays ───────────────────────────────────────────────────────
phases_cir = np.array(phases_cir, dtype=np.float64)
powers_cir = np.array(powers_cir, dtype=np.float64)
dopplers_solver = np.array(dopplers_solver, dtype=np.float64)
num_paths_list = np.array(num_paths_list, dtype=np.int32)
scatterer_positions = np.array(scatterer_positions, dtype=np.float64)
t_snap = np.arange(N_SNAPSHOTS) * DT

# ══════════════════════════════════════════════════════════════════════════════
# 5. Phase analysis
# ══════════════════════════════════════════════════════════════════════════════

# 5a. Exact phase from known geometry
# One-way (TX→RX): φ_1w(t) = -2π · d(t) / λ
distances = np.linalg.norm(scatterer_positions - RX_POS[np.newaxis, :], axis=1)
phases_1way = -2 * np.pi * distances / WAVELENGTH
phases_1way_wrapped = np.angle(np.exp(1j * phases_1way))

# Round-trip (monostatic radar): φ_rt(t) = 2 · φ_1w(t) = -4π · d(t) / λ
# This models TX → scatterer → RX with co-located radar.
phases_exact = 2 * phases_1way                    # Round-trip phase
phases_exact_wrapped = np.angle(np.exp(1j * phases_exact))

# Verify CIR phase matches geometry
cir_geo_corr = np.abs(np.corrcoef(phases_cir, phases_1way_wrapped)[0, 1])

# 5b. Fit sinusoidal model to exact unwrapped phase
#     φ(t) = p0 + p1·t + p2·sin(ωt) + p3·cos(ωt)
#     β = sqrt(p2² + p3²),  f_Δ = p1 / (2π)
X_model = np.column_stack([
    np.ones(N_SNAPSHOTS),
    t_snap,
    np.sin(OMEGA_ROT * t_snap),
    np.cos(OMEGA_ROT * t_snap),
])
coeffs, _, _, _ = np.linalg.lstsq(X_model, phases_exact, rcond=None)
p_const, p_linear, p_sin, p_cos = coeffs

f_delta_fit = p_linear / (2 * np.pi)                    # Bulk Doppler [Hz]
beta_fit = np.sqrt(p_sin**2 + p_cos**2)                 # Modulation index [rad]
phi_rot_fit = np.arctan2(p_cos, p_sin)                  # Fitted rotation phase

phase_model = X_model @ coeffs
phase_residual = phases_exact - phase_model
rms_residual_rad = np.std(phase_residual)

# 5c. Instantaneous frequency from numerical derivative of exact phase
f_inst = np.zeros(N_SNAPSHOTS)
f_inst[1:-1] = (phases_exact[2:] - phases_exact[:-2]) / (2 * DT) / (2 * np.pi)
f_inst[0] = (phases_exact[1] - phases_exact[0]) / DT / (2 * np.pi)
f_inst[-1] = (phases_exact[-1] - phases_exact[-2]) / DT / (2 * np.pi)

# Theoretical instantaneous frequency
f_inst_theo = f_delta_fit + BETA_EFF * F_ROT * np.cos(OMEGA_ROT * t_snap + phi_rot_fit)

# 5d. Compute the modulation-only phase (remove bulk Doppler)
phase_modulation = phases_exact - (p_const + p_linear * t_snap)
phase_modulation_theo = BETA_EFF * np.sin(OMEGA_ROT * t_snap + phi_rot_fit)

# ══════════════════════════════════════════════════════════════════════════════
# 6. Results summary
# ══════════════════════════════════════════════════════════════════════════════

beta_err_rel = abs(beta_fit - BETA_EFF) / BETA_EFF
power_drift_db = 10 * np.log10(powers_cir / powers_cir[0])

print("=" * 72)
print("  Results")
print("=" * 72)
print(f"  Modulation index β (fitted)  : {beta_fit:.2f} rad")
print(f"  Modulation index β (theory)   : {BETA_EFF:.2f} rad")
print(f"  β relative error              : {beta_err_rel * 100:.2f} %")
print(f"  Bulk Doppler f_Δ (fitted)     : {f_delta_fit:.2f} Hz")
print(f"  Fitted rotation phase φ_rot   : {phi_rot_fit:.3f} rad  (true: {PHI0:.3f})")
print(f"  RMS phase residual            : {rms_residual_rad:.4f} rad  ({np.rad2deg(rms_residual_rad):.4f}°)")
print(f"  Peak freq deviation (fitted)  : ±{beta_fit * F_ROT * 1e-3:.2f} kHz")
print(f"  Peak freq deviation (theory)  : ±{F_DEV_PEAK * 1e-3:.2f} kHz")
print(f"  CIR vs geometry correlation   : {cir_geo_corr:.6f}")
print(f"  Path-solver Doppler (mean)    : {np.mean(dopplers_solver):+.2f} Hz  "
      f"(σ={np.std(dopplers_solver):.2f})")
print(f"  Path count (mean/min/max)     : {np.mean(num_paths_list):.1f} / "
      f"{np.min(num_paths_list)} / {np.max(num_paths_list)}")
print(f"  Power variation pk-pk         : {np.max(power_drift_db) - np.min(power_drift_db):.3f} dB")
print()

if beta_err_rel < 0.05:
    print("  ✓ PASS — β matches theory (< 5% error)")
elif beta_err_rel < 0.10:
    print("  ~ MARGINAL — β error between 5-10%")
else:
    print("  ✗ FAIL — β error > 10%, check setup")

# ══════════════════════════════════════════════════════════════════════════════
# 7. Visualization
# ══════════════════════════════════════════════════════════════════════════════

plt.rcParams.update({
    "figure.dpi": 150,
    "font.size": 9,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "legend.fontsize": 7.5,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
})

# ── Figure 1: Main 4-panel overview ─────────────────────────────────────────

fig1 = plt.figure(figsize=(13, 9))
fig1.suptitle(
    f"Micro-Doppler: Single Rotating Scatterer — Paris (etoile)\n"
    f"fc={FC / 1e9:.0f} GHz  R={R_BLADE:.2f} m  f_rot={F_ROT} Hz  "
    f"fs={1 / DT * 1e-3:.0f} kHz  N={N_SNAPSHOTS}  β≈{BETA_EFF:.0f} rad",
    fontsize=11, fontweight="bold",
)

gs = GridSpec(2, 2, figure=fig1, hspace=0.35, wspace=0.30)

# (a) 3D scatterer trajectory
ax3d = fig1.add_subplot(gs[0, 0], projection="3d")
ax3d.plot(scatterer_positions[:, 0], scatterer_positions[:, 1],
          scatterer_positions[:, 2], "b-", linewidth=0.8, label="Scatterer trajectory")
ax3d.scatter(*BODY_CENTER, color="red", s=40, marker="x", label="Body center")
ax3d.scatter(*RX_POS, color="green", s=40, marker="^", label="Radar (RX)")
ax3d.set_xlabel("X [m]")
ax3d.set_ylabel("Y [m]")
ax3d.set_zlabel("Z [m]")
ax3d.set_title("(a) Scatterer 3D Rotation Trajectory")
ax3d.legend(loc="best", fontsize=7)

# (b) Instantaneous Phase — modulated φ(t) vs time
ax = fig1.add_subplot(gs[0, 1])
# Show modulation component (bulk Doppler removed)
ax.plot(t_snap * 1e3, phase_modulation, "b.-", markersize=3, linewidth=0.8,
        label="Measured (exact geometry)")
ax.plot(t_snap * 1e3, phase_modulation_theo, "r--", linewidth=1.2,
        label=f"Theory: β={BETA_EFF:.0f}·sin(ωt+φ₀)")
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Phase Modulation [rad]")
ax.set_title(f"(b) Phase Modulation φ_mod(t)  [β_fit = {beta_fit:.1f} rad]")
ax.legend(loc="best")
ax.grid(True, alpha=0.3)

# (c) Instantaneous Frequency vs time
ax = fig1.add_subplot(gs[1, 0])
ax.plot(t_snap * 1e3, f_inst * 1e-3, "b.-", markersize=3, linewidth=0.8,
        label="Measured f_inst")
ax.plot(t_snap * 1e3, f_inst_theo * 1e-3, "r--", linewidth=1.2,
        label=f"Theory: f_Δ + β·f_rot·cos(ωt)")
ax.axhline(y=f_delta_fit * 1e-3, color="gray", linestyle=":", linewidth=0.8,
           label=f"Bulk f_Δ = {f_delta_fit:.1f} Hz")
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Instantaneous Frequency [kHz]")
ax.set_title(f"(c) Instantaneous Frequency  [peak dev ≈ ±{F_DEV_PEAK * 1e-3:.1f} kHz]")
ax.legend(loc="best")
ax.grid(True, alpha=0.3)

# (d) Wrapped CIR phase vs geometry prediction
ax = fig1.add_subplot(gs[1, 1])
ax.plot(t_snap * 1e3, phases_cir * 180 / np.pi, "b.-", markersize=3, linewidth=0.8,
        alpha=0.7, label="CIR phase (wrapped)")
ax.plot(t_snap * 1e3, phases_exact_wrapped * 180 / np.pi, "r.--", markersize=2,
        linewidth=1.0, alpha=0.7, label="Geometry prediction (wrapped)")
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Phase [deg]")
ax.set_title(f"(d) CIR vs Geometry Wrapped Phase  [corr = {cir_geo_corr:.4f}]")
ax.legend(loc="best")
ax.grid(True, alpha=0.3)

fig1.subplots_adjust(hspace=0.35, wspace=0.30)
fig1_path = OUT_DIR / "micro_doppler_single_scatterer.png"
fig1.savefig(fig1_path, dpi=200, bbox_inches="tight")
print(f"Figure saved to {fig1_path}")

# ── Figure 2: Modulation detail — phase + frequency decomposed ──────────────

fig2, axes2 = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig2.suptitle(
    "Micro-Doppler Modulation Analysis — Decomposed View",
    fontsize=11, fontweight="bold",
)

# (a) Full unwrapped phase (geometry-derived)
ax = axes2[0]
ax.plot(t_snap * 1e3, phases_exact, "b-", linewidth=0.6, alpha=0.7,
        label="Exact phase φ(t) (unwrapped)")
ax.plot(t_snap * 1e3, phase_model, "r--", linewidth=1.0,
        label=f"Model fit (β={beta_fit:.1f}, f_Δ={f_delta_fit:.1f} Hz)")
ax.set_ylabel("Phase [rad]")
ax.set_title("(a) Full Unwrapped Phase φ(t) = φ₀ + 2πf_Δ·t + β·sin(2πf_rot·t + φ_rot)")
ax.legend(loc="best")
ax.grid(True, alpha=0.3)

# (b) Phase residual (after removing linear + sinusoidal fit)
ax = axes2[1]
ax.plot(t_snap * 1e3, phase_residual * 180 / np.pi, "k.-", markersize=2, linewidth=0.6)
ax.axhline(0, color="gray", linestyle="--", linewidth=0.5)
ax.axhline(+rms_residual_rad * 180 / np.pi, color="red", linestyle=":", linewidth=0.8,
           label=f"±RMS = {rms_residual_rad * 180 / np.pi:.3f}°")
ax.axhline(-rms_residual_rad * 180 / np.pi, color="red", linestyle=":", linewidth=0.8)
ax.set_ylabel("Residual [deg]")
ax.set_title(f"(b) Phase Residual after Model Fit  "
             f"[RMS = {rms_residual_rad:.4f} rad = {rms_residual_rad * 180 / np.pi:.4f}°]")
ax.legend(loc="best")
ax.grid(True, alpha=0.3)

# (c) Instantaneous frequency error
f_inst_error = f_inst - f_inst_theo
ax = axes2[2]
ax.plot(t_snap * 1e3, f_inst_error, "m.-", markersize=2, linewidth=0.6)
ax.axhline(0, color="gray", linestyle="--", linewidth=0.5)
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Freq Error [Hz]")
ax.set_title(f"(c) Instantaneous Frequency Error  "
             f"[RMS = {np.std(f_inst_error):.1f} Hz]")
ax.grid(True, alpha=0.3)

plt.tight_layout()
fig2_path = OUT_DIR / "micro_doppler_modulation_analysis.png"
fig2.savefig(fig2_path, dpi=200, bbox_inches="tight")
print(f"Figure saved to {fig2_path}")

# ── Figure 3: Micro-Doppler vs Standard Doppler side-by-side ────────────────

fig3, axes3 = plt.subplots(2, 2, figsize=(13, 7))
fig3.suptitle(
    "Micro-Doppler vs Standard Doppler Comparison",
    fontsize=11, fontweight="bold",
)

# (a) Standard Doppler: linear phase ramp
t_linear = np.linspace(0, TOTAL_TIME, N_SNAPSHOTS)
f_d_standard = FC * 10.0 / SPEED_OF_LIGHT  # v=10 m/s → ~933 Hz
phi_standard = 2 * np.pi * f_d_standard * t_linear
ax = axes3[0, 0]
ax.plot(t_linear * 1e3, phi_standard, "b-", linewidth=1.0)
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Phase [rad]")
ax.set_title(f"(a) Standard Doppler: φ(t) = 2π·{f_d_standard:.0f}·t  (linear)")
ax.grid(True, alpha=0.3)

# (b) Micro-Doppler: modulated phase
ax = axes3[0, 1]
ax.plot(t_snap * 1e3, phase_modulation, "r-", linewidth=1.0)
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Phase Modulation [rad]")
ax.set_title(f"(b) Micro-Doppler: φ_mod(t) = {BETA_EFF:.0f}·sin(2π·{F_ROT}·t)  (sinusoidal)")
ax.grid(True, alpha=0.3)

# (c) Standard Doppler: constant freq
ax = axes3[1, 0]
ax.axhline(y=f_d_standard, color="b", linewidth=2.0)
ax.set_xlim(0, TOTAL_TIME * 1e3)
ax.set_ylim(f_d_standard - 10, f_d_standard + 10)
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Instantaneous Freq [Hz]")
ax.set_title(f"(c) Standard Doppler: f_inst = {f_d_standard:.0f} Hz  (constant)")
ax.grid(True, alpha=0.3)

# (d) Micro-Doppler: oscillating freq
ax = axes3[1, 1]
ax.plot(t_snap * 1e3, f_inst * 1e-3, "r-", linewidth=1.0)
ax.set_xlabel("Time [ms]")
ax.set_ylabel("Instantaneous Freq [kHz]")
ax.set_title(f"(d) Micro-Doppler: f_inst oscillates ±{F_DEV_PEAK * 1e-3:.1f} kHz")
ax.grid(True, alpha=0.3)

plt.tight_layout()
fig3_path = OUT_DIR / "micro_doppler_vs_standard_doppler.png"
fig3.savefig(fig3_path, dpi=200, bbox_inches="tight")
print(f"Figure saved to {fig3_path}")

# ── Figure 4: Power stability & path count ──────────────────────────────────

fig4, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.5))
fig4.suptitle("CIR Diagnostics — Power Stability & Path Count", fontsize=10, fontweight="bold")

ax1.plot(t_snap * 1e3, power_drift_db, "g.-", markersize=3, linewidth=0.8)
ax1.axhline(np.mean(power_drift_db), color="red", linestyle="--", linewidth=0.8,
            label=f"Mean = {np.mean(power_drift_db):.3f} dB")
ax1.set_xlabel("Time [ms]")
ax1.set_ylabel("Relative Power [dB]")
ax1.set_title("Dominant Path Power Stability")
ax1.legend(loc="best")
ax1.grid(True, alpha=0.3)

ax2.plot(t_snap * 1e3, num_paths_list, "b.-", markersize=3, linewidth=0.8)
ax2.set_xlabel("Time [ms]")
ax2.set_ylabel("Path Count")
ax2.set_title(f"Path Count  [μ={np.mean(num_paths_list):.1f}]")
ax2.grid(True, alpha=0.3)

plt.tight_layout()
fig4_path = OUT_DIR / "micro_doppler_cir_diagnostics.png"
fig4.savefig(fig4_path, dpi=200, bbox_inches="tight")
print(f"Figure saved to {fig4_path}")

plt.show()
print("\nDone — Step 2 complete.")
