#!/usr/bin/env python3
"""Recreate a Fig. 2-style UAV micro-Doppler spectrogram pair.

Target paper:
  Sun et al., "Micro-Doppler Signature-Based Detection, Classification,
  and Localization of Small UAV With Long Short-Term Memory Neural Network",
  IEEE TGRS 2021.

The paper's Fig. 2 is a typical-MDS illustration, not a parameter-complete
reproducibility figure. The paper does specify the experimental radar/STFT
settings used later in the data set:
  - carrier centered at 915 MHz, 2 kHz bandwidth
  - 32 kHz sampling rate
  - 1 s MDS segments
  - 256-point Hamming window
  - 93.75% overlap

This script uses those settings and a low-frequency analytic propeller model
so the output has the same visual scale as Fig. 2: time 0..1 s and
micro-Doppler frequency +/-300 Hz.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.constants import c as SPEED_OF_LIGHT
from scipy.signal import spectrogram, windows

OUT_DIR = Path(__file__).resolve().parent.parent / "figures"
OUT_DIR.mkdir(exist_ok=True)
OUT_PATH = OUT_DIR / "paper_fig2_style_microhelicopter_vs_quadcopter.png"

# Paper experimental/STFT settings
FC = 915e6
FS = 32_000
T_OBS = 1.0
N_SAMPLES = int(FS * T_OBS)
N_WIN = 256
N_FFT = 1024  # zero-padding for a smoother Fig. 2-style display
OVERLAP_RATIO = 0.9375
N_OVERLAP = int(N_WIN * OVERLAP_RATIO)
FREQ_LIMIT_HZ = 300

RNG = np.random.default_rng(20260520)


@dataclass(frozen=True)
class Rotor:
    """A compact low-frequency propeller micro-Doppler component."""

    rotation_hz: float
    peak_doppler_hz: float
    phase_rad: float
    amplitude: float = 1.0
    direction: float = 1.0
    blade_flash_weight: float = 0.20
    n_blades: int = 2

    @property
    def implied_tip_speed_mps(self) -> float:
        # Bistatic factor/aspect are unknown in Fig. 2; this is the monostatic
        # upper-bound speed that would produce the selected peak Doppler.
        return self.peak_doppler_hz * SPEED_OF_LIGHT / (2 * FC)


def rotor_signal(t: np.ndarray, rotor: Rotor) -> np.ndarray:
    """Generate a complex baseband return for one propeller.

    Instantaneous frequency follows
        f_md(t) = direction * f_peak * cos(2*pi*f_rot*t + phase)
    and the integrated phase is
        phi(t) = direction * (f_peak/f_rot) * sin(2*pi*f_rot*t + phase).
    A weak blade-flash envelope creates the dense vertical striations visible
    in the paper-style spectrogram without changing the frequency axis scale.
    """
    angle = 2 * np.pi * rotor.rotation_hz * t + rotor.phase_rad
    beta = rotor.peak_doppler_hz / rotor.rotation_hz
    phase = rotor.direction * beta * np.sin(angle)
    flash = 1.0 + rotor.blade_flash_weight * np.cos(rotor.n_blades * angle) ** 8
    return rotor.amplitude * flash * np.exp(1j * phase)


def add_measurement_texture(signal: np.ndarray, snr_db: float = 18.0) -> np.ndarray:
    """Add mild amplitude fading and complex receiver noise."""
    n = len(signal)
    t = np.arange(n) / FS
    fading = 1.0 + 0.04 * np.sin(2 * np.pi * 2.2 * t + 0.5)
    fading += 0.025 * np.sin(2 * np.pi * 7.7 * t + 1.3)
    textured = signal * fading
    power = np.mean(np.abs(textured) ** 2)
    noise_power = power / (10 ** (snr_db / 10))
    noise = np.sqrt(noise_power / 2) * (RNG.standard_normal(n) + 1j * RNG.standard_normal(n))
    return textured + noise


def microhelicopter_signal(t: np.ndarray) -> np.ndarray:
    # One dominant propeller: cleaner and more periodic, matching the paper's
    # explanation that the microhelicopter MDS is less blurred.
    rotor = Rotor(rotation_hz=42.0, peak_doppler_hz=260.0, phase_rad=0.35, amplitude=1.0)
    body = 0.25 * np.exp(1j * 2 * np.pi * 5.0 * t)
    return add_measurement_texture(body + rotor_signal(t, rotor), snr_db=22.0)


def quadcopter_signal(t: np.ndarray) -> np.ndarray:
    # Four propellers, two clockwise and two anticlockwise. Slight differences
    # in rotation speed and phase intentionally blur the MDS, as described in
    # the paper text beside Fig. 2.
    rotors = [
        Rotor(39.0, 235.0, 0.10, 0.65, +1.0, 0.20),
        Rotor(41.5, 250.0, 1.65, 0.60, -1.0, 0.18),
        Rotor(43.0, 220.0, 3.25, 0.58, +1.0, 0.22),
        Rotor(40.5, 245.0, 4.80, 0.62, -1.0, 0.19),
    ]
    body = 0.18 * np.exp(1j * 2 * np.pi * 3.0 * t)
    combined = body + sum(rotor_signal(t, rotor) for rotor in rotors)
    return add_measurement_texture(combined, snr_db=18.0)


def compute_mds(signal: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    f_raw, t_spec, zxx = spectrogram(
        signal,
        fs=FS,
        window=windows.hamming(N_WIN, sym=False),
        nperseg=N_WIN,
        noverlap=N_OVERLAP,
        nfft=N_FFT,
        detrend=False,
        mode="complex",
        return_onesided=False,
    )
    freq = np.fft.fftshift(f_raw)
    spec = np.fft.fftshift(zxx, axes=0)
    s_db = 20 * np.log10(np.abs(spec) + 1e-12)
    s_db -= np.max(s_db)
    return freq, t_spec, s_db


def plot_pair() -> None:
    t = np.arange(N_SAMPLES) / FS
    signals = [microhelicopter_signal(t), quadcopter_signal(t)]
    titles = ["micro-helicopter", "quadcopter"]

    fig, axes = plt.subplots(1, 2, figsize=(8.2, 3.35), constrained_layout=True)
    for ax, sig, title in zip(axes, signals, titles):
        freq, t_spec, s_db = compute_mds(sig)
        mask = np.abs(freq) <= FREQ_LIMIT_HZ
        im = ax.pcolormesh(
            t_spec,
            freq[mask],
            s_db[mask, :] + 30,
            shading="gouraud",
            cmap="jet",
            vmin=-38,
            vmax=30,
        )
        ax.set_title(title, fontsize=12, fontweight="bold", pad=4)
        ax.set_xlabel("time (s)", fontsize=11)
        ax.set_ylabel("micro-Doppler frequency (Hz)", fontsize=11)
        ax.set_xlim(0, 1)
        ax.set_ylim(-FREQ_LIMIT_HZ, FREQ_LIMIT_HZ)
        ax.set_xticks([0, 0.5, 1.0])
        ax.set_xticklabels(["0", "0.5", "1"])
        ax.set_yticks([-300, -200, -100, 0, 100, 200, 300])
        cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.045)
        cbar.set_ticks([-30, -20, -10, 0, 10, 20, 30])
        cbar.ax.tick_params(labelsize=9)

    fig.savefig(OUT_PATH, dpi=220, bbox_inches="tight")
    print("=" * 72)
    print("Paper Fig.2-style MDS pair generated")
    print("=" * 72)
    print(f"Output: {OUT_PATH}")
    print(f"Radar/STFT: fc={FC/1e6:.0f} MHz, fs={FS/1e3:.0f} kHz, "
          f"window={N_WIN} Hamming, nfft={N_FFT}, overlap={OVERLAP_RATIO*100:.2f}%")
    print(f"Display range: +/-{FREQ_LIMIT_HZ} Hz, T={T_OBS:.1f} s")
    print("Implied monostatic tip-speed scale for selected peak Dopplers:")
    for name, peak in [("micro-helicopter", 260.0), ("quadcopter", 220.0)]:
        speed = peak * SPEED_OF_LIGHT / (2 * FC)
        print(f"  {name:<18s}: {peak:5.1f} Hz -> {speed:5.1f} m/s")


if __name__ == "__main__":
    plot_pair()
