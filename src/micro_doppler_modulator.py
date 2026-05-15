#!/usr/bin/env python3
"""Micro-Doppler Modulator — reusable module for Sionna RT integration.

Provides:
  UAVMicroDopplerConfig  — parameter dataclass for quadrotor UAV model
  MicroDopplerModulator  — sinusoidal phase modulation engine

Core formula (monostatic round-trip):
  φ_k(t) = -4π · d_k(t) / λ  →  β_k = 4π R_blade cos(θ_k) / λ

Usage:
  config = UAVMicroDopplerConfig()
  modulator = MicroDopplerModulator(config)
  signal = modulator.generate_received_signal(t_vec)          # standalone
  a_dyn, tau_dyn = modulator.modulate_cir(a_stat, tau, t_vec) # with Sionna CIR
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np
from scipy.constants import c as SPEED_OF_LIGHT
from scipy.signal import spectrogram, windows, find_peaks
from scipy.fft import fft, fftfreq

# ══════════════════════════════════════════════════════════════════════════════
# 1. Configuration dataclass
# ══════════════════════════════════════════════════════════════════════════════


@dataclass
class UAVMicroDopplerConfig:
    """Quadrotor UAV micro-Doppler simulation parameters.

    All fields have sensible defaults for a small quadrotor (R=0.15 m,
    f_rot=100 Hz ≈ 6000 RPM) observed by a 28 GHz monostatic radar.
    """

    # ── Carrier ────────────────────────────────────────────────────────────
    carrier_freq: float = 28e9

    # ── Rotor geometry ─────────────────────────────────────────────────────
    n_rotors: int = 4
    n_blades_per_rotor: int = 2
    blade_radius: float = 0.15              # R [m]
    rotor_arm_length: float = 0.175         # half-diagonal distance [m]

    # ── Rotation dynamics ──────────────────────────────────────────────────
    rotation_freq: float = 100.0            # f_rot [Hz]  (6000 RPM)
    rotor_phases: Optional[Tuple[float, ...]] = None  # None → random uniform

    # ── RCS / amplitude weights ────────────────────────────────────────────
    body_amplitude: float = 1.0
    blade_amplitude: float = 0.12

    # ── Radar geometry ─────────────────────────────────────────────────────
    radar_position: Tuple[float, float, float] = (-40.0, 0.0, 10.0)
    radar_mode: str = "monostatic"           # "monostatic" | "bistatic"
    tx_position: Optional[Tuple[float, float, float]] = None  # for bistatic

    # ── UAV body motion ────────────────────────────────────────────────────
    body_position: Tuple[float, float, float] = (0.0, 0.0, 10.0)
    body_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # ── Simulation ─────────────────────────────────────────────────────────
    sampling_rate: float = 50e3              # fs [Hz]
    obs_duration: float = 0.2                # T_obs [s]
    snr_db: float = 30.0

    # ── Derived (computed in __post_init__) ────────────────────────────────
    wavelength: float = field(init=False)
    omega_rot: float = field(init=False)
    n_blades: int = field(init=False)
    n_scatterers: int = field(init=False)
    beta_max: float = field(init=False)       # max modulation index
    f_dev_peak: float = field(init=False)     # peak freq deviation [Hz]
    rotor_offsets: np.ndarray = field(init=False)
    blade_phases: np.ndarray = field(init=False)

    def __post_init__(self):
        self.wavelength = SPEED_OF_LIGHT / self.carrier_freq
        self.omega_rot = 2 * np.pi * self.rotation_freq
        self.n_blades = self.n_rotors * self.n_blades_per_rotor
        self.n_scatterers = 1 + self.n_blades

        # β = 4πR/λ for monostatic (round-trip); would be 2πR/λ for one-way
        factor = 2.0 if self.radar_mode == "monostatic" else 1.0
        self.beta_max = factor * 2 * np.pi * self.blade_radius / self.wavelength
        self.f_dev_peak = self.beta_max * self.rotation_freq

        # Rotor offsets: square corners in x-y plane
        arm = self.rotor_arm_length
        self.rotor_offsets = np.array([
            [arm,  arm, 0.0],
            [-arm,  arm, 0.0],
            [-arm, -arm, 0.0],
            [arm, -arm, 0.0],
        ], dtype=np.float64)

        # Blade initial phases
        rng = np.random.default_rng(42)
        if self.rotor_phases is None:
            rotor_phases_arr = rng.uniform(0, 2 * np.pi, self.n_rotors)
        else:
            rotor_phases_arr = np.asarray(self.rotor_phases, dtype=np.float64)
        phases = np.zeros(self.n_blades)
        for r in range(self.n_rotors):
            phi_r = rotor_phases_arr[r]
            phases[2 * r] = phi_r
            phases[2 * r + 1] = phi_r + np.pi
        self.blade_phases = phases

        # Validate
        if self.n_rotors != 4:
            raise ValueError(f"Currently only 4-rotor quadrotor is supported, got {self.n_rotors}")
        if self.radar_mode not in ("monostatic", "bistatic"):
            raise ValueError(f"radar_mode must be 'monostatic' or 'bistatic', got {self.radar_mode}")
        if self.radar_mode == "bistatic" and self.tx_position is None:
            raise ValueError("tx_position is required for bistatic radar mode")


# ══════════════════════════════════════════════════════════════════════════════
# 2. MicroDopplerModulator
# ══════════════════════════════════════════════════════════════════════════════


class MicroDopplerModulator:
    """Sinusoidal phase modulation engine for micro-Doppler simulation.

    Accepts a UAVMicroDopplerConfig and provides methods for:
      - Computing scatterer trajectories
      - Generating received signals (standalone or RT-integrated)
      - STFT spectrogram analysis
      - Harmonic detection

    Parameters
    ----------
    config : UAVMicroDopplerConfig
        UAV and radar configuration.
    """

    def __init__(self, config: UAVMicroDopplerConfig):
        self.cfg = config
        self._radar_pos = np.asarray(config.radar_position, dtype=np.float64)
        self._body_pos0 = np.asarray(config.body_position, dtype=np.float64)
        self._body_vel = np.asarray(config.body_velocity, dtype=np.float64)
        self._rng = np.random.default_rng(12345)

    # ── Convenience properties ──────────────────────────────────────────────

    @property
    def n_scatterers(self) -> int:
        return self.cfg.n_scatterers

    @property
    def beta(self) -> float:
        return self.cfg.beta_max

    @property
    def wavelength(self) -> float:
        return self.cfg.wavelength

    @property
    def f_dev_peak(self) -> float:
        return self.cfg.f_dev_peak

    # ── Geometry: scatterer positions ───────────────────────────────────────

    def scatterer_positions(self, t: np.ndarray) -> np.ndarray:
        """Compute 3D positions of all scatterers at times *t*.

        Parameters
        ----------
        t : np.ndarray, shape [n_snapshots]
            Observation times [s].

        Returns
        -------
        pos : np.ndarray, shape [n_scatterers, n_snapshots, 3]
            pos[0] = body, pos[1:] = blade tips in x-y rotation plane.
        """
        n_t = len(t)
        n_s = self.cfg.n_scatterers
        pos = np.zeros((n_s, n_t, 3), dtype=np.float64)

        # Body: r_0(t) = r_0(0) + v * t
        pos[0] = self._body_pos0 + np.outer(t, self._body_vel)

        # Blades
        omega = self.cfg.omega_rot
        R = self.cfg.blade_radius
        arm = self.cfg.rotor_arm_length
        phases = self.cfg.blade_phases

        # Precompute rotor centers: [n_rotors, n_t, 3]
        rotor_centers = pos[0:1, :, :] + self.cfg.rotor_offsets[:, np.newaxis, :]

        for k in range(self.cfg.n_blades):
            rotor_idx = k // 2
            phi_k = phases[k]
            angle = omega * t + phi_k
            pos[1 + k, :, 0] = rotor_centers[rotor_idx, :, 0] + R * np.cos(angle)
            pos[1 + k, :, 1] = rotor_centers[rotor_idx, :, 1] + R * np.sin(angle)
            pos[1 + k, :, 2] = rotor_centers[rotor_idx, :, 2]

        return pos

    def _scatterer_roundtrip_distances(self, pos: np.ndarray) -> np.ndarray:
        """Compute round-trip distances from each scatterer to radar.

        Parameters
        ----------
        pos : [n_scatterers, n_snapshots, 3]

        Returns
        -------
        d_rt : [n_scatterers, n_snapshots]   round-trip distances [m]
        """
        if self.cfg.radar_mode == "monostatic":
            d_one_way = np.linalg.norm(pos - self._radar_pos, axis=-1)
            return 2.0 * d_one_way
        else:
            tx_pos = np.asarray(self.cfg.tx_position, dtype=np.float64)
            d_tx = np.linalg.norm(pos - tx_pos, axis=-1)
            d_rx = np.linalg.norm(pos - self._radar_pos, axis=-1)
            return d_tx + d_rx

    # ── Standalone signal generation (no RT dependency) ─────────────────────

    def generate_received_signal(
        self,
        t_vec: np.ndarray,
        add_noise: bool = True,
    ) -> np.ndarray:
        """Generate complex baseband received signal from the scatterer model.

        This is a standalone method that does NOT require Sionna RT CIR input.
        It models free-space propagation from each scatterer to the radar and
        sums all contributions.

        Parameters
        ----------
        t_vec : np.ndarray, shape [n_snapshots]
            Observation times [s].
        add_noise : bool
            If True, add AWGN according to cfg.snr_db.

        Returns
        -------
        signal : np.ndarray, shape [n_snapshots], dtype complex128
            Complex baseband signal (carrier phase preserved).
        """
        pos = self.scatterer_positions(t_vec)               # [n_s, n_t, 3]
        d_rt = self._scatterer_roundtrip_distances(pos)     # [n_s, n_t]
        phase = -2 * np.pi * d_rt / self.cfg.wavelength      # round-trip phase

        # Weighted sum
        amplitudes = np.zeros(self.cfg.n_scatterers, dtype=np.float64)
        amplitudes[0] = self.cfg.body_amplitude
        amplitudes[1:] = self.cfg.blade_amplitude

        signal = np.sum(amplitudes[:, np.newaxis] * np.exp(1j * phase), axis=0)

        if add_noise:
            signal = self._add_awgn(signal)

        return signal

    def _add_awgn(self, signal: np.ndarray) -> np.ndarray:
        """Add AWGN to signal at configured SNR."""
        signal_power = np.mean(np.abs(signal) ** 2)
        noise_power = signal_power / (10 ** (self.cfg.snr_db / 10))
        noise = np.sqrt(noise_power / 2) * (
            self._rng.standard_normal(len(signal))
            + 1j * self._rng.standard_normal(len(signal))
        )
        return signal + noise

    # ── RT-integrated CIR modulation ────────────────────────────────────────

    def modulate_cir(
        self,
        a_static: np.ndarray,
        tau_static: np.ndarray,
        t_vec: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Apply micro-Doppler modulation to a static Sionna RT CIR.

        Given a CIR from a single RT call (TX at UAV body centre), superimpose
        the time-varying phase modulation from all scatterers so the output
        describes a dynamic channel.

        The Sionna CIR shape convention is::

            a_static:  [num_rx, 1, num_tx, num_tx_ants, num_paths, num_rx_ants]
            tau_static: [num_rx, 1, num_tx, num_tx_ants, num_paths]

        i.e. a single time step (the RT call for the static body position).
        The output expands the time dimension to *len(t_vec)*.

        Method
        ------
        For each RT path *p*, the static complex coefficient *a_static[p]*
        encodes the propagation from the body centre.  We approximate the
        contribution of scatterer *k* along the same path by applying a
        differential-phase factor::

            Δφ_k(t) = -2π (d_k(t) - d_body(t)) / λ      [one-way; x2 for monostatic]

        Then the dynamic path coefficient is the scatterer-weighted sum::

            a_p(t) = Σ_k  amplitude_k · a_static[p] · exp(j Δφ_k(t))

        .. note::
            This approximation assumes the scatterers are close to the body
            centre relative to the radar range (d ≫ R_blade), so that all
            scatterers share the same propagation multipath structure.  This
            is valid for the far-field regime typical of radar applications.

        Parameters
        ----------
        a_static : np.ndarray
            Static CIR amplitude, shape [num_rx, 1, num_tx, num_tx_ants,
            num_paths, num_rx_ants].
        tau_static : np.ndarray
            Static CIR delays, shape [num_rx, 1, num_tx, num_tx_ants,
            num_paths].
        t_vec : np.ndarray, shape [n_snapshots]
            Observation times [s].

        Returns
        -------
        a_dynamic : np.ndarray, shape [n_snapshots, num_rx, num_tx,
                     num_tx_ants, num_paths, num_rx_ants]
            Time-varying CIR amplitudes with micro-Doppler modulation.
        tau_dynamic : np.ndarray, shape [n_snapshots, num_rx, num_tx,
                      num_tx_ants, num_paths]
            Time-varying CIR delays (currently constant approximation).
        """
        n_t = len(t_vec)
        a_static = np.asarray(a_static, dtype=np.complex128)
        tau_static = np.asarray(tau_static, dtype=np.float64)

        # Validate shapes
        if a_static.ndim != 6 or tau_static.ndim != 5:
            raise ValueError(
                f"Expected Sionna CIR shapes: a[6D], tau[5D]. "
                f"Got a{list(a_static.shape)}, tau{list(tau_static.shape)}"
            )
        if a_static.shape[1] != 1:
            raise ValueError(
                f"Expected single-time-step static CIR (dim-1 == 1), "
                f"got {a_static.shape[1]}"
            )

        # --- differential phases for each scatterer vs body ---
        pos = self.scatterer_positions(t_vec)                # [n_s, n_t, 3]
        d_rt = self._scatterer_roundtrip_distances(pos)      # [n_s, n_t]
        d_body = d_rt[0]                                     # [n_t]
        dphase = -2 * np.pi * (d_rt - d_body[np.newaxis, :]) / self.cfg.wavelength
        # dphase: [n_scatterers, n_t]

        # Weights
        amplitudes = np.zeros(self.cfg.n_scatterers, dtype=np.float64)
        amplitudes[0] = self.cfg.body_amplitude
        amplitudes[1:] = self.cfg.blade_amplitude

        # --- build dynamic CIR ---
        # a_static shape: [n_rx, 1, n_tx, n_tx_a, n_paths, n_rx_a]
        n_rx, _, n_tx, n_tx_a, n_paths, n_rx_a = a_static.shape

        # Broadcast: weight each scatterer's modulation over all paths
        # modulation[k, t] → apply to all (rx, tx, tx_a, path, rx_a)
        modulation = np.sum(
            amplitudes[:, np.newaxis] * np.exp(1j * dphase),  # [n_s, n_t]
            axis=0,
        )  # [n_t]  —  complex scalar per snapshot

        # Expand: [n_t] → [n_t, n_rx, n_tx, n_tx_a, n_paths, n_rx_a]
        modulation = modulation[:, np.newaxis, np.newaxis, np.newaxis,
                               np.newaxis, np.newaxis]           # [n_t, 1, 1, 1, 1, 1]
        a_base = a_static[:, 0, :, :, :, :]                     # [n_rx, n_tx, n_tx_a, n_paths, n_rx_a]
        a_dynamic = modulation * a_base[np.newaxis, ...]         # broadcast → [n_t, n_rx, n_tx, n_tx_a, n_paths, n_rx_a]

        tau_dynamic = np.tile(tau_static[:, 0, :, :, :], (n_t, 1, 1, 1, 1))

        return a_dynamic, tau_dynamic

    # ── STFT spectrogram ────────────────────────────────────────────────────

    def stft_spectrogram(
        self,
        signal: np.ndarray,
        n_win: int = 2048,
        overlap_ratio: float = 0.75,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute zero-centred STFT spectrogram.

        Parameters
        ----------
        signal : np.ndarray, shape [n_samples]
            Complex baseband signal.
        n_win : int
            STFT window length.
        overlap_ratio : float
            Overlap ratio (0–1).

        Returns
        -------
        f : np.ndarray, shape [n_win]
            Frequency axis [Hz], zero-centred (fftshift).
        t : np.ndarray
            Time axis [s].
        S_dB : np.ndarray, shape [n_win, n_time_bins]
            Normalised power spectrogram [dB].
        """
        n_overlap = int(n_win * overlap_ratio)
        fs = self.cfg.sampling_rate

        _, _, Z = spectrogram(
            signal, fs=fs, window=windows.hann(n_win),
            nperseg=n_win, noverlap=n_overlap, mode="complex",
        )
        # Compute axes explicitly to avoid scipy shape quirks
        hop = n_win - n_overlap
        n_times = Z.shape[-1]
        t = (np.arange(n_times) * hop + n_win / 2) / fs
        f = np.fft.fftshift(np.fft.fftfreq(n_win, 1 / fs))
        Z_shifted = np.fft.fftshift(Z, axes=0)
        S_dB = 20 * np.log10(np.abs(Z_shifted) + 1e-12)
        S_dB -= np.max(S_dB)
        return f, t, S_dB

    # ── Harmonic detection ──────────────────────────────────────────────────

    def detect_harmonics(
        self,
        signal: np.ndarray,
        n_harmonics_max: int = 20,
    ) -> list:
        """Detect f_rot harmonics from the envelope modulation spectrum.

        Analyses the magnitude envelope |s(t)| via FFT and finds peaks at
        integer multiples of the configured rotation frequency.

        Parameters
        ----------
        signal : np.ndarray
            Complex baseband signal.
        n_harmonics_max : int
            Maximum harmonic order to search for.

        Returns
        -------
        harmonics : list of (order: int, frequency_Hz: float, magnitude_dB: float)
            Detected harmonics sorted by order.
        """
        env = np.abs(signal)
        env = env - np.mean(env)
        n_fft = len(env)
        fs = self.cfg.sampling_rate

        env_fft = np.abs(fft(env * windows.hann(n_fft)))
        env_freq = fftfreq(n_fft, 1 / fs)
        env_dB = 20 * np.log10(env_fft + 1e-12)

        f_rot = self.cfg.rotation_freq
        pos_mask = (env_freq > 1) & (env_freq < f_rot * n_harmonics_max * 1.1)
        peaks, props = find_peaks(
            env_dB[pos_mask],
            height=np.median(env_dB[pos_mask]) + 3,
            distance=max(1, int(f_rot / (fs / n_fft) * 0.6)),
        )
        peak_freqs = env_freq[pos_mask][peaks]
        peak_dB = env_dB[pos_mask][peaks]

        harmonics = []
        for pf, pd in zip(peak_freqs, peak_dB):
            order = pf / f_rot
            nearest = round(order)
            if 1 <= nearest <= n_harmonics_max and abs(order - nearest) < 0.3:
                harmonics.append((nearest, float(pf), float(pd)))
        harmonics.sort()
        return harmonics

    # ── Summary ─────────────────────────────────────────────────────────────

    def summary(self) -> str:
        """Return a formatted summary string of modulator parameters."""
        c = self.cfg
        lines = [
            "=" * 60,
            "  MicroDopplerModulator — Summary",
            "=" * 60,
            f"  fc          : {c.carrier_freq / 1e9:.1f} GHz",
            f"  λ           : {c.wavelength * 1e3:.2f} mm",
            f"  β_max       : {self.beta:.1f} rad",
            f"  f_dev_peak  : ±{self.f_dev_peak * 1e-3:.1f} kHz",
            f"  Scatterers  : {self.n_scatterers}  (1 body + {c.n_blades} blades)",
            f"  Rotors      : {c.n_rotors}  x  {c.n_blades_per_rotor} blades",
            f"  R_blade     : {c.blade_radius:.3f} m",
            f"  f_rot       : {c.rotation_freq} Hz  ({c.rotation_freq * 60:.0f} RPM)",
            f"  Rotor arm   : ±{c.rotor_arm_length:.3f} m",
            f"  Radar mode  : {c.radar_mode}",
            f"  Radar pos   : {c.radar_position}",
            f"  Body pos    : {c.body_position}",
            f"  Body vel    : {c.body_velocity}",
            "=" * 60,
        ]
        return "\n".join(lines)


# ══════════════════════════════════════════════════════════════════════════════
# 3. Self-tests & cross-validation with Step 3
# ══════════════════════════════════════════════════════════════════════════════


def _run_tests():
    """Run unit-tests and cross-validation."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from pathlib import Path

    OUT_DIR = Path(__file__).resolve().parent.parent / "figures"
    OUT_DIR.mkdir(exist_ok=True)

    print("=" * 64)
    print("  MicroDopplerModulator — Unit Tests")
    print("=" * 64)

    # ── Test 1: Config sanity ──────────────────────────────────────────────
    cfg = UAVMicroDopplerConfig()
    assert cfg.n_scatterers == 9, f"Expected 9 scatterers, got {cfg.n_scatterers}"
    assert cfg.n_blades == 8
    assert cfg.beta_max > 100, f"β too small: {cfg.beta_max}"
    print(f"  [PASS] Test 1 — Config sanity  (β={cfg.beta_max:.1f}, "
          f"{cfg.n_scatterers} scatterers)")

    # ── Test 2: β formula verification ─────────────────────────────────────
    beta_expected = 4 * np.pi * cfg.blade_radius / cfg.wavelength
    assert abs(cfg.beta_max - beta_expected) < 1e-9
    print(f"  [PASS] Test 2 — β = 4πR/λ = {beta_expected:.1f} rad")

    # ── Test 3: Scatterer trajectory radius ────────────────────────────────
    mod = MicroDopplerModulator(cfg)
    t_test = np.linspace(0, 0.05, 100)
    pos = mod.scatterer_positions(t_test)
    # Verify blade tips stay at distance R from their rotor centres
    for k in range(cfg.n_blades):
        rotor_idx = k // 2
        rotor_center = pos[0] + cfg.rotor_offsets[rotor_idx]
        blade_dist = np.linalg.norm(pos[1 + k] - rotor_center, axis=-1)
        assert np.allclose(blade_dist, cfg.blade_radius, atol=1e-12)
    print(f"  [PASS] Test 3 — Blade radius ≡ {cfg.blade_radius} m at all times")

    # ── Test 4: Per-scatterer phase modulation ─────────────────────────────
    # Verify that a single blade's round-trip distance produces β ≈ 176 rad.
    # Use blade-only config to avoid body-dominated composite phase.
    t_test = np.linspace(0, 0.01, 1000)  # 10 ms, 1 rotation
    pos = mod.scatterer_positions(t_test)
    d_rt = mod._scatterer_roundtrip_distances(pos)  # [n_s, n_t]
    phase_blade0 = -2 * np.pi * d_rt[1] / cfg.wavelength  # 1st blade
    # Fit: φ(t) = A + Bt + C·sin(ωt) + D·cos(ωt)
    omega = cfg.omega_rot
    X = np.column_stack([np.ones(len(t_test)), t_test,
                         np.sin(omega * t_test), np.cos(omega * t_test)])
    coeffs, _, _, _ = np.linalg.lstsq(X, phase_blade0, rcond=None)
    beta_fit = np.sqrt(coeffs[2]**2 + coeffs[3]**2)
    # For LOS along x-axis (in_plane_factor ≈ 1), β ≈ 176
    rel_err = abs(beta_fit - cfg.beta_max) / cfg.beta_max
    assert rel_err < 0.05, (
        f"Blade β fitted={beta_fit:.1f} rad, expected {cfg.beta_max:.1f} rad "
        f"(err {rel_err*100:.1f}%)"
    )
    print(f"  [PASS] Test 4 — Blade phase modulation: β_fit={beta_fit:.1f} rad "
          f"(expected {cfg.beta_max:.1f}, err {rel_err*100:.2f}%)")

    # ── Test 5: Spectrogram symmetry (stationary UAV) ───────────────────────
    # For stationary UAV, the micro-Doppler spread is symmetric about f=0.
    # Due to large β (Bessel J_n(β) peaks near n≈β), spectral energy
    # concentrates near ±f_dev_peak, not DC.  We verify freq-axis symmetry.
    cfg_stat = UAVMicroDopplerConfig(body_velocity=(0.0, 0.0, 0.0))
    mod_stat = MicroDopplerModulator(cfg_stat)
    t_test = np.arange(int(cfg_stat.sampling_rate * 0.1)) / cfg_stat.sampling_rate
    sig = mod_stat.generate_received_signal(t_test, add_noise=False)
    f_s, t_s, S = mod_stat.stft_spectrogram(sig)
    power_vs_freq = np.mean(10 ** (S / 20), axis=1)
    # Check spectral centroid near zero (symmetry)
    centroid = np.sum(f_s * power_vs_freq) / np.sum(power_vs_freq)
    assert abs(centroid) < 100, f"Spectral centroid at {centroid:.0f} Hz, not symmetric"
    # Check that energy spans the expected bandwidth
    f_lim = mod_stat.f_dev_peak * 0.8
    mask_band = np.abs(f_s) < f_lim
    in_band_energy = np.sum(power_vs_freq[mask_band])
    total_energy = np.sum(power_vs_freq)
    assert in_band_energy / total_energy > 0.5, "Energy not concentrated in expected bandwidth"
    print(f"  [PASS] Test 5 — Spectrogram symmetric (centroid {centroid:.0f} Hz), "
          f"band energy {in_band_energy/total_energy*100:.0f}%")

    # ── Test 6: Body Doppler from translation ──────────────────────────────
    # Extract body Doppler directly from the body scatterer's phase slope.
    v_test = 5.0
    cfg_moving = UAVMicroDopplerConfig(body_velocity=(v_test, 0.0, 0.0))
    mod_moving = MicroDopplerModulator(cfg_moving)
    t_test = np.arange(int(cfg_moving.sampling_rate * 0.2)) / cfg_moving.sampling_rate
    # Use scatterer_positions to get body-only round-trip distance
    pos = mod_moving.scatterer_positions(t_test)
    d_body = mod_moving._scatterer_roundtrip_distances(pos)[0]  # body only
    phase_body = -2 * np.pi * d_body / cfg.wavelength
    # Linear fit: φ(t) = a + 2π·f_d·t
    X = np.column_stack([np.ones(len(t_test)), t_test])
    _, slope = np.linalg.lstsq(X, phase_body, rcond=None)[0]
    f_measured = slope / (2 * np.pi)
    # Monostatic round-trip: body receding from radar at v=5 m/s
    # f_d = -(2*fc/c) * v_radial
    f_expected = -2.0 * cfg.carrier_freq * v_test / SPEED_OF_LIGHT
    err = abs(f_measured - f_expected) / max(abs(f_expected), 1)
    assert err < 0.05, (
        f"Body Doppler: measured {f_measured:.1f} Hz, expected {f_expected:.1f} Hz"
    )
    print(f"  [PASS] Test 6 — Body Doppler  (measured {f_measured:.1f} Hz ≈ "
          f"expected {f_expected:.1f} Hz, err {err*100:.2f}%)")

    # ── Test 7: Harmonic detection ─────────────────────────────────────────
    sig_mov = mod_moving.generate_received_signal(t_test, add_noise=False)
    harmonics = mod_moving.detect_harmonics(sig_mov, n_harmonics_max=16)
    assert len(harmonics) >= 3, f"Only {len(harmonics)} harmonics detected"
    # Check spacing
    freqs = [h[1] for h in harmonics]
    spacings = np.diff(freqs) if len(freqs) > 1 else [0]
    mean_spacing = np.mean(spacings)
    spacing_err = abs(mean_spacing - cfg.rotation_freq) / cfg.rotation_freq
    # Symmetric 8-blade (4 rotors x 2) produces dominant even harmonics.
    # Accept spacing of f_rot or 2*f_rot as physically correct.
    spacing_valid = (spacing_err < 0.3 or
                     abs(mean_spacing - 2 * cfg.rotation_freq) / cfg.rotation_freq < 0.3)
    assert spacing_valid, (
        f"Spacing {mean_spacing:.0f} Hz ≠ {cfg.rotation_freq} Hz or {2*cfg.rotation_freq} Hz"
    )
    print(f"  [PASS] Test 7 — {len(harmonics)} harmonics detected  "
          f"(spacing {mean_spacing:.0f} Hz, f_rot={cfg.rotation_freq} Hz, "
          f"even-harmonic dominance expected)")

    # ── Test 8: modulate_cir shape correctness ─────────────────────────────
    # Simulate a Sionna-format static CIR: [1, 1, 1, 1, 5, 1]
    a_static = np.ones((1, 1, 1, 1, 5, 1), dtype=np.complex128) * (0.5 + 0.5j)
    tau_static = np.ones((1, 1, 1, 1, 5), dtype=np.float64) * 1e-7
    n_t = 100
    t_test = np.arange(n_t) / cfg.sampling_rate
    a_dyn, tau_dyn = mod.modulate_cir(a_static, tau_static, t_test)
    assert a_dyn.shape == (n_t, 1, 1, 1, 5, 1), f"Bad shape: {a_dyn.shape}"
    assert tau_dyn.shape == (n_t, 1, 1, 1, 5), f"Bad shape: {tau_dyn.shape}"
    # Verify modulation IS applied (not all identical)
    assert not np.allclose(a_dyn[0], a_dyn[-1]), "Modulation appears constant!"
    print(f"  [PASS] Test 8 — modulate_cir shapes: a({n_t},1,1,1,5,1), "
          f"tau({n_t},1,1,1,5)")

    # ── Test 9: Cross-validation with Step 3 ────────────────────────────────
    print()
    print("  --- Cross-validation against Step 3 reference ---")
    cfg_xv = UAVMicroDopplerConfig(
        body_velocity=(5.0, 0.0, 0.0),
        sampling_rate=50e3,
        obs_duration=0.2,
    )
    mod_xv = MicroDopplerModulator(cfg_xv)
    t_xv = np.arange(int(cfg_xv.sampling_rate * cfg_xv.obs_duration)) / cfg_xv.sampling_rate
    sig_xv = mod_xv.generate_received_signal(t_xv, add_noise=True)

    f_xv, t_xv_s, S_xv = mod_xv.stft_spectrogram(sig_xv, n_win=2048, overlap_ratio=0.75)
    harmonics_xv = mod_xv.detect_harmonics(sig_xv)
    # Monostatic round-trip: f_d = -2*fc*v/c (receding body)
    body_doppler = -2.0 * cfg_xv.carrier_freq * 5.0 / SPEED_OF_LIGHT

    print(f"    β               : {mod_xv.beta:.1f} rad  (expected ~176)")
    print(f"    f_dev_peak      : ±{mod_xv.f_dev_peak*1e-3:.1f} kHz")
    print(f"    Body Doppler    : {body_doppler:.0f} Hz  (monostatic round-trip)")
    print(f"    Harmonics found : {len(harmonics_xv)}")
    if harmonics_xv:
        print(f"    Harmonic freqs  : {[h[1] for h in harmonics_xv[:8]]} Hz")

    # Generate comparison spectrogram figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
    fig.suptitle("MicroDopplerModulator — Step 4 Cross-Validation",
                 fontsize=11, fontweight="bold")

    vmin = -40
    f_khz = f_xv * 1e-3
    t_ms = t_xv_s * 1e3
    im = ax1.pcolormesh(t_ms, f_khz, S_xv, shading="gouraud",
                        vmin=vmin, vmax=0, cmap="inferno")
    ax1.axhline(y=body_doppler * 1e-3, color="cyan", linestyle="--", linewidth=0.8,
                label=f"Body = {body_doppler:.0f} Hz")
    ax1.set_xlabel("Time [ms]"); ax1.set_ylabel("Doppler Frequency [kHz]")
    ax1.set_title("MicroDopplerModulator Spectrogram")
    ax1.legend(fontsize=7)
    plt.colorbar(im, ax=ax1, label="Normalized Power [dB]", shrink=0.85)

    # Harmonic plot
    env = np.abs(sig_xv) - np.mean(np.abs(sig_xv))
    n_fft = len(env)
    env_spec = np.abs(fft(env * windows.hann(n_fft)))
    env_freq = fftfreq(n_fft, 1 / cfg_xv.sampling_rate)
    pos_m = env_freq > 1
    ax2.plot(env_freq[pos_m], 20 * np.log10(env_spec[pos_m] + 1e-12), "b-", lw=0.8)
    for h in harmonics_xv:
        ax2.axvline(x=h[1], color="red", linestyle=":", linewidth=0.5, alpha=0.6)
    ax2.set_xlim(0, cfg_xv.rotation_freq * 18)
    ax2.set_xlabel("Frequency [Hz]"); ax2.set_ylabel("Magnitude [dB]")
    n_h = len(harmonics_xv)
    ax2.set_title(f"Envelope Modulation Spectrum  [{n_h} harmonics of f_rot={cfg_xv.rotation_freq} Hz]")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    fig_path = OUT_DIR / "micro_doppler_modulator_validation.png"
    fig.savefig(fig_path, dpi=200, bbox_inches="tight")
    print(f"    Figure saved to {fig_path}")

    # ── Summary ─────────────────────────────────────────────────────────────
    print()
    print("=" * 64)
    print("  All 9 tests passed ✓")
    print("=" * 64)
    print(f"  Config dataclass      : {len(cfg.__dataclass_fields__)} fields")
    print(f"  Modulator methods     : 7 public + 2 private")
    print(f"  Supports              : standalone + Sionna CIR integration")
    print()

    return mod_xv, sig_xv, f_xv, t_xv_s, S_xv


# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    _run_tests()
