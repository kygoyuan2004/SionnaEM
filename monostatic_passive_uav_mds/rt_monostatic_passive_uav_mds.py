#!/usr/bin/env python3
"""Monostatic passive UAV MDS validation with per-snapshot Sionna RT.

This script keeps the BS/Radar as the only active transmitter. The UAV body and
blade samples are represented as passive probe receivers. At every snapshot, the
body translates, blade samples rotate around rotor centers that follow the body,
and Sionna RT recomputes the one-way BS -> scatterer channel. A monostatic
round-trip return is approximated by reciprocity:

    s_k(t) = weight_k * h_k(t)^2

where h_k(t) is the coherent one-way CIR sum for scatterer k. Squaring h_k gives
round-trip path loss and phase while avoiding the old active blade-TX shortcut.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.constants import c as SPEED_OF_LIGHT
from scipy.signal import spectrogram, windows

import sionna
from sionna.rt import PathSolver, PlanarArray, Receiver, Transmitter, load_scene


ROOT_DIR = Path(__file__).resolve().parent
OUTPUT_FIG_DIR = ROOT_DIR / "outputs" / "figures"
OUTPUT_DATA_DIR = ROOT_DIR / "outputs" / "data"
OUTPUT_LOG_DIR = ROOT_DIR / "outputs" / "logs"


@dataclass(frozen=True)
class PassiveUavMdsConfig:
    """Configuration for passive monostatic UAV MDS validation."""

    scene: str = "floor_wall"
    snapshots: int = 1024
    fs: float = 2000.0
    fc: float = 28e9
    max_depth: int = 0
    cir_sampling_frequency: float = 122.88e6

    bs_position: tuple[float, float, float] = (-20.0, 0.0, 3.0)
    body_position0: tuple[float, float, float] = (0.0, 0.0, 3.0)
    body_speed: float = 1.0
    body_direction: tuple[float, float, float] = (1.0, 0.0, 0.0)

    n_rotors: int = 4
    n_blades_per_rotor: int = 2
    points_per_blade: int = 3
    blade_radius: float = 0.02
    rotor_arm_length: float = 0.08
    rotation_freq: float = 20.0

    body_weight: float = 1.0
    blade_weight: float = 0.18
    snr_db: float = 30.0
    add_noise: bool = True

    stft_win: int = 256
    stft_overlap_ratio: float = 0.75
    seed: int = 12345

    output: str = str(OUTPUT_FIG_DIR / "monostatic_passive_uav_mds.png")
    data_output: str = str(OUTPUT_DATA_DIR / "monostatic_passive_uav_mds.npz")
    log_output: str = str(OUTPUT_LOG_DIR / "monostatic_passive_uav_mds_summary.txt")

    @property
    def wavelength(self) -> float:
        return SPEED_OF_LIGHT / self.fc

    @property
    def dt(self) -> float:
        return 1.0 / self.fs

    @property
    def t_vec(self) -> np.ndarray:
        return np.arange(self.snapshots, dtype=np.float64) * self.dt

    @property
    def nyquist(self) -> float:
        return 0.5 * self.fs

    @property
    def body_velocity(self) -> np.ndarray:
        direction = np.asarray(self.body_direction, dtype=np.float64)
        norm = np.linalg.norm(direction)
        if norm < 1e-12:
            return np.zeros(3, dtype=np.float64)
        return self.body_speed * direction / norm

    @property
    def n_blades(self) -> int:
        return self.n_rotors * self.n_blades_per_rotor

    @property
    def n_blade_points(self) -> int:
        return self.n_blades * self.points_per_blade

    @property
    def n_scatterers(self) -> int:
        return 1 + self.n_blade_points

    @property
    def stft_overlap(self) -> int:
        return int(self.stft_win * self.stft_overlap_ratio)

    @property
    def stft_df(self) -> float:
        return self.fs / self.stft_win


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Per-snapshot monostatic passive UAV MDS validation with Sionna RT."
    )
    parser.add_argument("--scene", choices=["floor_wall", "etoile"], default="floor_wall")
    parser.add_argument("--snapshots", type=int, default=1024)
    parser.add_argument("--fs", type=float, default=2000.0)
    parser.add_argument("--fc", type=float, default=28e9)
    parser.add_argument("--max-depth", type=int, default=0)
    parser.add_argument("--body-speed", type=float, default=1.0)
    parser.add_argument("--blade-radius", type=float, default=0.02)
    parser.add_argument("--rotation-freq", type=float, default=20.0)
    parser.add_argument("--points-per-blade", type=int, default=3)
    parser.add_argument("--body-weight", type=float, default=1.0)
    parser.add_argument("--blade-weight", type=float, default=0.18)
    parser.add_argument("--stft-win", type=int, default=256)
    parser.add_argument("--snr-db", type=float, default=30.0)
    parser.add_argument("--no-noise", action="store_true")
    parser.add_argument("--output", default=str(OUTPUT_FIG_DIR / "monostatic_passive_uav_mds.png"))
    parser.add_argument("--data-output", default=str(OUTPUT_DATA_DIR / "monostatic_passive_uav_mds.npz"))
    parser.add_argument("--log-output", default=str(OUTPUT_LOG_DIR / "monostatic_passive_uav_mds_summary.txt"))
    return parser.parse_args()


def config_from_args(args: argparse.Namespace) -> PassiveUavMdsConfig:
    return PassiveUavMdsConfig(
        scene=args.scene,
        snapshots=args.snapshots,
        fs=args.fs,
        fc=args.fc,
        max_depth=args.max_depth,
        body_speed=args.body_speed,
        blade_radius=args.blade_radius,
        rotation_freq=args.rotation_freq,
        points_per_blade=max(1, args.points_per_blade),
        body_weight=args.body_weight,
        blade_weight=args.blade_weight,
        stft_win=args.stft_win,
        snr_db=args.snr_db,
        add_noise=not args.no_noise,
        output=args.output,
        data_output=args.data_output,
        log_output=args.log_output,
    )


def ensure_output_dirs() -> None:
    for path in (OUTPUT_FIG_DIR, OUTPUT_DATA_DIR, OUTPUT_LOG_DIR):
        path.mkdir(parents=True, exist_ok=True)


def rotor_offsets(cfg: PassiveUavMdsConfig) -> np.ndarray:
    arm = cfg.rotor_arm_length
    if cfg.n_rotors != 4:
        raise ValueError("This validation script currently supports exactly 4 rotors.")
    return np.asarray(
        [
            [arm, arm, 0.0],
            [-arm, arm, 0.0],
            [-arm, -arm, 0.0],
            [arm, -arm, 0.0],
        ],
        dtype=np.float64,
    )


def body_position(cfg: PassiveUavMdsConfig, t: float) -> np.ndarray:
    return np.asarray(cfg.body_position0, dtype=np.float64) + cfg.body_velocity * float(t)


def scatterer_state(cfg: PassiveUavMdsConfig, t: float) -> dict[str, np.ndarray | list[str]]:
    """Return scatterer positions, velocities, weights, labels, and radii."""
    bs_pos = np.asarray(cfg.bs_position, dtype=np.float64)
    body_pos = body_position(cfg, t)
    body_vel = cfg.body_velocity
    offsets = rotor_offsets(cfg)

    positions = [body_pos]
    velocities = [body_vel]
    weights = [cfg.body_weight]
    labels = ["body"]
    rotor_ids = [-1]
    radii = [0.0]
    rotor_centers = body_pos[np.newaxis, :] + offsets

    omega = 2.0 * np.pi * cfg.rotation_freq
    radial_samples = (
        np.linspace(1.0, cfg.points_per_blade, cfg.points_per_blade, dtype=np.float64)
        / cfg.points_per_blade
        * cfg.blade_radius
    )
    per_point_weight = cfg.blade_weight / max(1, cfg.points_per_blade)

    for rotor_idx, center in enumerate(rotor_centers):
        rotor_phase = rotor_idx * np.pi / 2.0
        for blade_idx in range(cfg.n_blades_per_rotor):
            blade_phase = 2.0 * np.pi * blade_idx / cfg.n_blades_per_rotor
            angle = omega * float(t) + rotor_phase + blade_phase
            cos_a = np.cos(angle)
            sin_a = np.sin(angle)
            for rho in radial_samples:
                rel = np.asarray([rho * cos_a, rho * sin_a, 0.0], dtype=np.float64)
                rot_vel = np.asarray([-omega * rho * sin_a, omega * rho * cos_a, 0.0], dtype=np.float64)
                positions.append(center + rel)
                velocities.append(body_vel + rot_vel)
                weights.append(per_point_weight)
                labels.append(f"rotor{rotor_idx}_blade{blade_idx}_rho{rho:.4f}")
                rotor_ids.append(rotor_idx)
                radii.append(float(rho))

    positions_arr = np.asarray(positions, dtype=np.float64)
    velocities_arr = np.asarray(velocities, dtype=np.float64)
    los = positions_arr - bs_pos[np.newaxis, :]
    los_norm = np.linalg.norm(los, axis=1)
    los_unit = los / np.maximum(los_norm[:, np.newaxis], 1e-12)
    one_way_range_rate = np.sum(velocities_arr * los_unit, axis=1)
    monostatic_freq = -2.0 * one_way_range_rate / cfg.wavelength

    return {
        "positions": positions_arr,
        "velocities": velocities_arr,
        "weights": np.asarray(weights, dtype=np.float64),
        "labels": labels,
        "rotor_ids": np.asarray(rotor_ids, dtype=np.int32),
        "radii": np.asarray(radii, dtype=np.float64),
        "rotor_centers": rotor_centers,
        "monostatic_freq": monostatic_freq,
    }


def scatterer_trajectories(cfg: PassiveUavMdsConfig) -> dict[str, np.ndarray | list[str]]:
    states = [scatterer_state(cfg, float(t)) for t in cfg.t_vec]
    positions = np.stack([s["positions"] for s in states], axis=0)
    velocities = np.stack([s["velocities"] for s in states], axis=0)
    rotor_centers = np.stack([s["rotor_centers"] for s in states], axis=0)
    monostatic_freq = np.stack([s["monostatic_freq"] for s in states], axis=0)
    first = states[0]
    return {
        "positions": positions,
        "velocities": velocities,
        "rotor_centers": rotor_centers,
        "weights": first["weights"],
        "labels": first["labels"],
        "rotor_ids": first["rotor_ids"],
        "radii": first["radii"],
        "monostatic_freq": monostatic_freq,
    }


def theory_metrics(cfg: PassiveUavMdsConfig) -> dict[str, float]:
    traj = scatterer_trajectories(cfg)
    freqs = np.asarray(traj["monostatic_freq"], dtype=np.float64)
    body_freq = float(np.median(freqs[:, 0]))
    if freqs.shape[1] > 1:
        blade_deviation = freqs[:, 1:] - freqs[:, 0:1]
        micro_peak = float(np.max(np.abs(blade_deviation)))
    else:
        micro_peak = 0.0
    return {
        "body_doppler_hz": body_freq,
        "micro_doppler_peak_hz": micro_peak,
        "f_min_expected_hz": body_freq - micro_peak,
        "f_max_expected_hz": body_freq + micro_peak,
        "nyquist_hz": cfg.nyquist,
        "stft_df_hz": cfg.stft_df,
    }


def load_rt_scene(cfg: PassiveUavMdsConfig):
    if cfg.scene == "etoile":
        scene = load_scene(sionna.rt.scene.etoile)
    else:
        scene = load_scene(sionna.rt.scene.floor_wall)

    scene.frequency = cfg.fc
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
    return scene


def setup_scene(cfg: PassiveUavMdsConfig):
    scene = load_rt_scene(cfg)
    scene.add(
        Transmitter(
            name="bs_radar",
            position=list(cfg.bs_position),
            orientation=[0.0, 0.0, 0.0],
            power_dbm=0.0,
        )
    )

    initial = scatterer_state(cfg, 0.0)
    labels = initial["labels"]
    positions = np.asarray(initial["positions"], dtype=np.float64)
    for idx, (label, pos) in enumerate(zip(labels, positions)):
        scene.add(
            Receiver(
                name=f"probe_{idx:02d}_{label}",
                position=pos.tolist(),
                orientation=[0.0, 0.0, 0.0],
            )
        )
    return scene, [f"probe_{idx:02d}_{label}" for idx, label in enumerate(labels)]


def update_probe_positions(scene, probe_names: list[str], positions: np.ndarray) -> None:
    for name, pos in zip(probe_names, positions):
        scene.get(name).position = np.asarray(pos, dtype=np.float64).tolist()


def run_solver(scene, solver: PathSolver, cfg: PassiveUavMdsConfig) -> np.ndarray:
    paths = solver(
        scene,
        max_depth=cfg.max_depth,
        los=True,
        specular_reflection=True,
        diffuse_reflection=False,
        diffraction=False,
        edge_diffraction=False,
        refraction=False,
        synthetic_array=False,
    )
    a, _ = paths.cir(
        sampling_frequency=cfg.cir_sampling_frequency,
        normalize_delays=False,
        out_type="numpy",
    )
    return np.asarray(a, dtype=np.complex128)


def one_way_channels_by_rx(a: np.ndarray, n_receivers: int) -> np.ndarray:
    """Sum one-way Sionna CIR paths for each probe receiver."""
    if a.ndim != 6:
        raise ValueError(f"Expected 6-D CIR coefficient tensor, got shape {a.shape}")
    if a.shape[0] < n_receivers:
        raise ValueError(f"Expected at least {n_receivers} receivers, got {a.shape[0]}")
    return np.sum(a[:n_receivers, 0, 0, 0, :, 0], axis=-1)


def monostatic_return(one_way_h: np.ndarray, weights: np.ndarray) -> complex:
    return complex(np.sum(weights * one_way_h * one_way_h))


def add_awgn(signal: np.ndarray, snr_db: float, rng: np.random.Generator) -> np.ndarray:
    signal_power = float(np.mean(np.abs(signal) ** 2))
    if signal_power <= 0.0:
        return signal.copy()
    noise_power = signal_power / (10.0 ** (snr_db / 10.0))
    noise = np.sqrt(noise_power / 2.0) * (
        rng.standard_normal(signal.shape) + 1j * rng.standard_normal(signal.shape)
    )
    return signal + noise


def compute_spectrogram(sig: np.ndarray, cfg: PassiveUavMdsConfig) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n_win = min(cfg.stft_win, len(sig))
    if n_win < 8:
        raise ValueError("Need at least 8 samples for STFT.")
    n_overlap = min(int(n_win * cfg.stft_overlap_ratio), n_win - 1)
    _, _, z = spectrogram(
        sig,
        fs=cfg.fs,
        window=windows.hann(n_win),
        nperseg=n_win,
        noverlap=n_overlap,
        mode="complex",
        return_onesided=False,
    )
    f_centered = np.fft.fftshift(np.fft.fftfreq(n_win, 1.0 / cfg.fs))
    z = np.fft.fftshift(z, axes=0)
    s_db = 20.0 * np.log10(np.abs(z) + 1e-12)
    s_db -= np.max(s_db)
    hop = n_win - n_overlap
    t_spec = (np.arange(z.shape[1], dtype=np.float64) * hop + n_win / 2.0) / cfg.fs
    return f_centered, t_spec, s_db


def frequency_profile_metrics(f_stft: np.ndarray, s_db: np.ndarray, threshold_db: float = -35.0) -> dict[str, float]:
    profile = np.max(s_db, axis=1)
    peak_idx = int(np.argmax(profile))
    active = np.flatnonzero(profile >= threshold_db)
    if len(active) == 0:
        f_min = f_max = float(f_stft[peak_idx])
    else:
        f_min = float(f_stft[active[0]])
        f_max = float(f_stft[active[-1]])
    return {
        "peak_frequency_hz": float(f_stft[peak_idx]),
        "support_min_hz": f_min,
        "support_max_hz": f_max,
        "support_threshold_db": threshold_db,
    }


def run_simulation(cfg: PassiveUavMdsConfig, verbose: bool = True) -> dict[str, Any]:
    if cfg.stft_win > cfg.snapshots:
        raise ValueError("stft_win must be <= snapshots.")
    if cfg.points_per_blade < 1:
        raise ValueError("points_per_blade must be >= 1.")

    rng = np.random.default_rng(cfg.seed)
    scene, probe_names = setup_scene(cfg)
    if len(scene.transmitters) != 1:
        raise RuntimeError(f"Expected exactly one transmitter, got {len(scene.transmitters)}")

    solver = PathSolver()
    traj = scatterer_trajectories(cfg)
    positions = np.asarray(traj["positions"], dtype=np.float64)
    weights = np.asarray(traj["weights"], dtype=np.float64)
    signal_clean = np.zeros(cfg.snapshots, dtype=np.complex128)
    one_way_h = np.zeros((cfg.snapshots, cfg.n_scatterers), dtype=np.complex128)
    path_counts = np.zeros((cfg.snapshots, cfg.n_scatterers), dtype=np.int32)

    start = time.perf_counter()
    for idx in range(cfg.snapshots):
        update_probe_positions(scene, probe_names, positions[idx])
        a = run_solver(scene, solver, cfg)
        h = one_way_channels_by_rx(a, cfg.n_scatterers)
        one_way_h[idx] = h
        path_counts[idx, :] = a.shape[4]
        signal_clean[idx] = monostatic_return(h, weights)
        if verbose and (idx + 1) % max(1, cfg.snapshots // 4) == 0:
            print(f"  snapshot {idx + 1:4d}/{cfg.snapshots}")

    elapsed = time.perf_counter() - start
    signal = add_awgn(signal_clean, cfg.snr_db, rng) if cfg.add_noise else signal_clean.copy()
    f_stft, t_stft, s_db = compute_spectrogram(signal, cfg)
    theory = theory_metrics(cfg)
    profile = frequency_profile_metrics(f_stft, s_db)

    return {
        "config": cfg,
        "signal": signal,
        "signal_clean": signal_clean,
        "one_way_h": one_way_h,
        "f_stft": f_stft,
        "t_stft": t_stft,
        "S_dB": s_db,
        "positions": positions,
        "velocities": np.asarray(traj["velocities"], dtype=np.float64),
        "rotor_centers": np.asarray(traj["rotor_centers"], dtype=np.float64),
        "weights": weights,
        "labels": traj["labels"],
        "rotor_ids": np.asarray(traj["rotor_ids"], dtype=np.int32),
        "radii": np.asarray(traj["radii"], dtype=np.float64),
        "monostatic_freq_theory": np.asarray(traj["monostatic_freq"], dtype=np.float64),
        "path_counts": path_counts,
        "theory": theory,
        "profile": profile,
        "elapsed_s": elapsed,
    }


def serialisable_config(cfg: PassiveUavMdsConfig) -> dict[str, Any]:
    data = asdict(cfg)
    data["body_velocity"] = cfg.body_velocity.tolist()
    data["wavelength"] = cfg.wavelength
    data["nyquist"] = cfg.nyquist
    data["stft_df"] = cfg.stft_df
    data["n_scatterers"] = cfg.n_scatterers
    return data


def save_outputs(result: dict[str, Any]) -> None:
    ensure_output_dirs()
    cfg: PassiveUavMdsConfig = result["config"]
    data_path = Path(cfg.data_output)
    data_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        data_path,
        signal=result["signal"],
        signal_clean=result["signal_clean"],
        f_stft=result["f_stft"],
        t_stft=result["t_stft"],
        S_dB=result["S_dB"],
        positions=result["positions"],
        velocities=result["velocities"],
        rotor_centers=result["rotor_centers"],
        weights=result["weights"],
        rotor_ids=result["rotor_ids"],
        radii=result["radii"],
        monostatic_freq_theory=result["monostatic_freq_theory"],
        path_counts=result["path_counts"],
        theory_json=np.asarray(json.dumps(result["theory"], indent=2)),
        profile_json=np.asarray(json.dumps(result["profile"], indent=2)),
        config_json=np.asarray(json.dumps(serialisable_config(cfg), indent=2)),
    )

    plot_results(result, Path(cfg.output))
    write_summary(result, Path(cfg.log_output))


def plot_results(result: dict[str, Any], output_path: Path) -> None:
    cfg: PassiveUavMdsConfig = result["config"]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    f = result["f_stft"]
    t = result["t_stft"]
    s_db = result["S_dB"]
    positions = result["positions"]
    theory = result["theory"]
    profile = np.max(s_db, axis=1)

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(
        "Monostatic passive UAV MDS from per-snapshot Sionna RT\n"
        "BS is the only TX; UAV body/blade samples are passive probe scatterers",
        fontsize=12,
        fontweight="bold",
    )
    gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.25)

    ax = fig.add_subplot(gs[0, 0])
    im = ax.pcolormesh(t * 1e3, f, s_db, shading="auto", cmap="turbo", vmin=-55, vmax=0)
    ax.axhline(theory["body_doppler_hz"], color="white", linestyle="--", linewidth=1.0, label="body Doppler")
    ax.axhline(theory["f_min_expected_hz"], color="white", linestyle=":", linewidth=0.8)
    ax.axhline(theory["f_max_expected_hz"], color="white", linestyle=":", linewidth=0.8)
    ax.set_title("STFT / Micro-Doppler spectrogram")
    ax.set_xlabel("Time [ms]")
    ax.set_ylabel("Doppler frequency [Hz]")
    ax.legend(loc="upper right")
    fig.colorbar(im, ax=ax, label="Normalised magnitude [dB]")

    ax = fig.add_subplot(gs[0, 1])
    ax.plot(f, profile, color="#1f77b4", linewidth=1.0)
    ax.axvline(theory["body_doppler_hz"], color="black", linestyle="--", linewidth=1.0, label="body Doppler")
    ax.axvline(theory["f_min_expected_hz"], color="#d62728", linestyle=":", linewidth=1.0, label="body ± mD peak")
    ax.axvline(theory["f_max_expected_hz"], color="#d62728", linestyle=":", linewidth=1.0)
    ax.set_title("Doppler profile")
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Max STFT magnitude [dB]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    ax = fig.add_subplot(gs[1, 0])
    body = positions[:, 0, :]
    first_blade = positions[:, 1, :]
    rotor0 = result["rotor_centers"][:, 0, :]
    bs_pos = np.asarray(cfg.bs_position, dtype=np.float64)
    ax.plot(body[:, 0], body[:, 1], color="black", linewidth=1.4, label="body")
    ax.plot(rotor0[:, 0], rotor0[:, 1], color="purple", linewidth=1.2, label="rotor center 0")
    ax.plot(first_blade[:, 0], first_blade[:, 1], color="#d62728", linewidth=1.0, label="first blade sample")
    ax.scatter([bs_pos[0]], [bs_pos[1]], marker="^", color="green", label="BS/radar")
    ax.set_title("Body and blade sample trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    ax = fig.add_subplot(gs[1, 1])
    phase = np.unwrap(np.angle(result["signal_clean"]))
    inst_freq = np.gradient(phase, 1.0 / cfg.fs) / (2.0 * np.pi)
    ax.plot(cfg.t_vec * 1e3, inst_freq, color="#ff7f0e", linewidth=0.9)
    ax.axhline(cfg.nyquist, color="black", linestyle=":", linewidth=0.8)
    ax.axhline(-cfg.nyquist, color="black", linestyle=":", linewidth=0.8)
    ax.set_title("Instantaneous frequency diagnostic")
    ax.set_xlabel("Time [ms]")
    ax.set_ylabel("Frequency [Hz]")
    ax.grid(True, alpha=0.3)

    fig.text(
        0.5,
        0.01,
        f"fc={cfg.fc/1e9:.1f} GHz, fs={cfg.fs:.0f} Hz, snapshots={cfg.snapshots}, "
        f"R={cfg.blade_radius:.3f} m, f_rot={cfg.rotation_freq:.1f} Hz, "
        f"points/blade={cfg.points_per_blade}, max_depth={cfg.max_depth}",
        ha="center",
        fontsize=9,
    )
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def write_summary(result: dict[str, Any], output_path: Path) -> None:
    cfg: PassiveUavMdsConfig = result["config"]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    theory = result["theory"]
    profile = result["profile"]
    lines = [
        "Monostatic passive UAV MDS summary",
        "=" * 48,
        f"scene                 : {cfg.scene}",
        f"fc                    : {cfg.fc/1e9:.3f} GHz",
        f"wavelength            : {cfg.wavelength*1e3:.3f} mm",
        f"snapshots/fs          : {cfg.snapshots} @ {cfg.fs:.1f} Hz",
        f"nyquist               : +/-{cfg.nyquist:.2f} Hz",
        f"stft_win/df           : {cfg.stft_win} / {cfg.stft_df:.2f} Hz",
        f"body speed            : {cfg.body_speed:.3f} m/s",
        f"rotation freq         : {cfg.rotation_freq:.3f} Hz",
        f"blade radius          : {cfg.blade_radius:.4f} m",
        f"points per blade      : {cfg.points_per_blade}",
        f"scatterers            : {cfg.n_scatterers}",
        f"single transmitter    : bs_radar",
        f"body Doppler theory   : {theory['body_doppler_hz']:.3f} Hz",
        f"mD peak theory        : {theory['micro_doppler_peak_hz']:.3f} Hz",
        f"expected support      : [{theory['f_min_expected_hz']:.3f}, {theory['f_max_expected_hz']:.3f}] Hz",
        f"STFT peak             : {profile['peak_frequency_hz']:.3f} Hz",
        f"STFT support          : [{profile['support_min_hz']:.3f}, {profile['support_max_hz']:.3f}] Hz "
        f"@ {profile['support_threshold_db']:.1f} dB",
        f"mean path count       : {np.mean(result['path_counts']):.2f}",
        f"elapsed               : {result['elapsed_s']:.2f} s",
        f"noise enabled         : {cfg.add_noise}",
        f"snr_db                : {cfg.snr_db:.1f}",
        "",
        "Configuration JSON",
        json.dumps(serialisable_config(cfg), indent=2),
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def print_header(cfg: PassiveUavMdsConfig) -> None:
    theory = theory_metrics(cfg)
    print("=" * 72)
    print("  Monostatic passive UAV MDS via per-snapshot Sionna RT")
    print("=" * 72)
    print(f"  scene          : {cfg.scene}")
    print(f"  fc/lambda      : {cfg.fc/1e9:.1f} GHz / {cfg.wavelength*1e3:.2f} mm")
    print(f"  snapshots/fs   : {cfg.snapshots} @ {cfg.fs:.0f} Hz")
    print(f"  Nyquist        : +/-{cfg.nyquist:.1f} Hz")
    print(f"  body speed     : {cfg.body_speed:.2f} m/s")
    print(f"  rotor/blades   : {cfg.n_rotors} x {cfg.n_blades_per_rotor}, points/blade={cfg.points_per_blade}")
    print(f"  R/f_rot        : {cfg.blade_radius:.3f} m / {cfg.rotation_freq:.1f} Hz")
    print(f"  body Doppler   : {theory['body_doppler_hz']:.1f} Hz")
    print(f"  mD peak        : +/-{theory['micro_doppler_peak_hz']:.1f} Hz")
    print(f"  expected band  : [{theory['f_min_expected_hz']:.1f}, {theory['f_max_expected_hz']:.1f}] Hz")
    print(f"  STFT df        : {cfg.stft_df:.1f} Hz")
    if abs(theory["f_min_expected_hz"]) > cfg.nyquist or abs(theory["f_max_expected_hz"]) > cfg.nyquist:
        print("  WARNING: expected Doppler support exceeds snapshot Nyquist.")
    print()


def main() -> None:
    args = parse_args()
    cfg = config_from_args(args)
    ensure_output_dirs()
    print_header(cfg)
    result = run_simulation(cfg, verbose=True)
    save_outputs(result)
    print()
    print(f"Saved figure : {cfg.output}")
    print(f"Saved data   : {cfg.data_output}")
    print(f"Saved summary: {cfg.log_output}")


if __name__ == "__main__":
    main()
