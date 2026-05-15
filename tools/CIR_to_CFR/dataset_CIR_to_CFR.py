# -*- coding: utf-8 -*-
"""
cir_to_cfr.py

Load per-antenna CIR dataset (.npz) and reconstruct full CFR on-the-fly.

Expected NPZ keys (from your generator):
  - user_position: [N, 3] float32
  - a_full       : [N, Nr, L] complex64
  - tau_full     : [N, Nr, L] float32   (seconds)
  - pilot_mask   : [T, K] bool (optional for training)
  - fc           : float64 (Hz) (optional)
  - scs          : float64 (Hz) (optional)
  - K            : int32  (optional)
  - L_keep       : int32  (optional)
  - num_paths_eff: [N] int32 (optional)
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass
from typing import Iterator, Tuple, Optional, Dict, Any


@dataclass
class CIRDataset:
    user_position: np.ndarray   # [N, 3] float32
    a_full: np.ndarray          # [N, Nr, L] complex64/complex128
    tau_full: np.ndarray        # [N, Nr, L] float32/float64
    pilot_mask: Optional[np.ndarray]  # [T, K] bool
    fc: float                   # Hz
    scs: float                  # Hz
    K: int
    L_keep: int
    num_paths_eff: Optional[np.ndarray]  # [N] int32


def _assert(cond: bool, msg: str):
    if not cond:
        raise ValueError(msg)


def _subcarrier_offsets(K: int, scs: float) -> np.ndarray:
    """
    OFDM subcarrier frequency offsets (baseband) in Hz, centered around 0.

    Matches typical OFDM indexing:
      offsets = (k - K/2) * scs for even K
    => symmetric around 0, with no DC special-case removal (you can mask DC later if desired).
    """
    _assert(K > 0, f"K must be positive, got K={K}")
    k = np.arange(K, dtype=np.float64)
    offsets = (k - (K // 2)) * float(scs)
    return offsets  # [K]

def subcarrier_frequency_offsets(K: int, scs: float) -> np.ndarray:
    return _subcarrier_offsets(K, scs)  # [K] in Hz (Δf)

def subcarrier_frequencies_abs(fc: float, K: int, scs: float) -> np.ndarray:
    """
    Absolute RF frequencies f_k = fc + offset_k.
    Returns [K] float64.
    """
    offsets = _subcarrier_offsets(K, scs)
    return float(fc) + offsets


def load_cir_dataset(npz_path: str,
                     override_fc: Optional[float] = None,
                     override_scs: Optional[float] = None,
                     override_K: Optional[int] = None) -> CIRDataset:
    """
    Load dataset from NPZ and do strict shape/type checks.
    """
    data = np.load(npz_path, allow_pickle=False)

    _assert("user_position" in data, "NPZ missing key: user_position")
    _assert("a_full" in data, "NPZ missing key: a_full")
    _assert("tau_full" in data, "NPZ missing key: tau_full")

    #START——REVISED
    #=================================================
    user_position = data["user_position"]
    a_full = data["a_full"]
    tau_full = data["tau_full"]

    # --------- (1) copy to avoid modifying np.load mmap buffers ----------
    a_full  = a_full.copy()
    tau_full = tau_full.copy()

    # --------- (2) invalid_tau: tau<0 (covers -1 sentinel and any negative jitter) ----------
    invalid_tau = (tau_full < 0.0)

    # IMPORTANT: zero both at invalid entries (prevents garbage a with tau=-1)
    a_full[invalid_tau] = 0.0 + 0.0j
    tau_full[invalid_tau] = 0.0

    # --------- (3) valid mask: exclude padding paths (a==0) and keep only physical paths ----------
    eps = 1e-12
    valid = (np.abs(a_full) > eps)   # bool [N, Nr, L]

    # --------- (4) shared shift per sample: subtract min tau over ALL antennas+paths but ONLY valid ----------
    # We need min over axis (Nr,L) for each sample n
    N = tau_full.shape[0]
    min_tau = np.zeros((N, 1, 1), dtype=tau_full.dtype)

    # If a sample has no valid paths at all, keep min_tau=0 and it remains all zeros
    has_valid = np.any(valid, axis=(1,2))  # [N]

    # compute min tau among valid entries for each sample
    # do it in a safe loop (N is fine) to avoid tricky masked reductions
    for n in range(N):
        if has_valid[n]:
            min_tau[n,0,0] = np.min(tau_full[n][valid[n]])
        else:
            min_tau[n,0,0] = 0.0

    tau_full = tau_full - min_tau

    # numerical safety: clip tiny negatives from float arithmetic
    tau_full = np.maximum(tau_full, 0.0).astype(np.float32, copy=False)
    a_full   = a_full.astype(np.complex64, copy=False)

    # OPTIONAL: keep valid/has_valid for debugging if you want (not required)
    # print("has_valid ratio:", np.mean(has_valid))

    print("has_valid ratio:", np.mean(has_valid))
    #=================================================
    #END-REVISED
    pilot_mask = data["pilot_mask"] if "pilot_mask" in data else None
    num_paths_eff = data["num_paths_eff"] if "num_paths_eff" in data else None

    # Metadata (use override if provided)
    fc = float(override_fc) if override_fc is not None else float(data["fc"]) if "fc" in data else None
    scs = float(override_scs) if override_scs is not None else float(data["scs"]) if "scs" in data else None
    K = int(override_K) if override_K is not None else int(data["K"]) if "K" in data else None
    L_keep = int(data["L_keep"]) if "L_keep" in data else a_full.shape[-1]

    _assert(fc is not None, "fc not found in NPZ and override_fc not provided")
    _assert(scs is not None, "scs not found in NPZ and override_scs not provided")
    _assert(K is not None, "K not found in NPZ and override_K not provided")

    # ---- shape checks ----
    _assert(user_position.ndim == 2 and user_position.shape[1] == 3,
            f"user_position must be [N,3], got {user_position.shape}")
    _assert(a_full.ndim == 3, f"a_full must be [N,Nr,L], got {a_full.shape}")
    _assert(tau_full.ndim == 3, f"tau_full must be [N,Nr,L], got {tau_full.shape}")
    _assert(a_full.shape == tau_full.shape,
            f"a_full shape {a_full.shape} must match tau_full shape {tau_full.shape}")

    N, Nr, L = a_full.shape
    _assert(user_position.shape[0] == N,
            f"user_position N={user_position.shape[0]} mismatch with a_full N={N}")
    _assert(L_keep == L,
            f"L_keep metadata ({L_keep}) mismatch with a_full last dim L={L}")

    if pilot_mask is not None:
        _assert(pilot_mask.ndim == 2 and pilot_mask.shape[1] == K,
                f"pilot_mask must be [T,K] with K={K}, got {pilot_mask.shape}")

    if num_paths_eff is not None:
        _assert(num_paths_eff.shape[0] == N,
                f"num_paths_eff must be [N], got {num_paths_eff.shape}")

    # ---- type checks ----
    _assert(np.iscomplexobj(a_full), "a_full must be complex dtype")
    _assert(np.issubdtype(tau_full.dtype, np.floating), "tau_full must be float dtype")

    # tau must be non-negative in propagation; allow zeros for padding
    #_assert(np.all(tau_full >= 0.0), "tau_full contains negative delays (should be >=0)")

    return CIRDataset(
        user_position=user_position.astype(np.float32, copy=False),
        a_full=a_full.astype(np.complex64, copy=False),
        tau_full=tau_full.astype(np.float32, copy=False),
        pilot_mask=pilot_mask,
        fc=fc,
        scs=scs,
        K=K,
        L_keep=L_keep,
        num_paths_eff=num_paths_eff
    )


def cfr_from_cir_single(a_mL: np.ndarray,
                        tau_mL: np.ndarray,
                        f_off: np.ndarray,
                        num_paths_eff: Optional[int] = None) -> np.ndarray:
    """
    Reconstruct CFR for ONE sample.

    Inputs:
      a_mL    : [Nr, L] complex64
      tau_mL  : [Nr, L] float32 (seconds)
      f_abs   : [K] float64/float32 (Hz)
      num_paths_eff : optional effective paths count (ignore padded zeros beyond this)

    Output:
      H_km : [K, Nr] complex64
    """
    _assert(a_mL.ndim == 2, f"a_mL must be [Nr,L], got {a_mL.shape}")
    _assert(tau_mL.shape == a_mL.shape, "tau_mL must have same shape as a_mL")
    _assert(f_off.ndim == 1, f"f_abs must be [K], got {f_off.shape}")

    Nr, L = a_mL.shape
    K = f_off.shape[0]

    L_eff = int(num_paths_eff) if num_paths_eff is not None else L
    L_eff = max(0, min(L_eff, L))

    # Use only effective paths (optional)
    a_use = a_mL[:, :L_eff]          # [Nr, L_eff]
    tau_use = tau_mL[:, :L_eff]      # [Nr, L_eff]

    # Compute phase: exp(-j 2pi f_k tau_m,l)
    # We want output [K, Nr]:
    #   H[k,m] = sum_l a[m,l] * exp(-j2pi f[k] * tau[m,l])
    #
    # Broadcasting strategy:
    #   f_abs[:, None, None] -> [K, 1, 1]
    #   tau_use[None, :, :]  -> [1, Nr, L_eff]
    # => phase -> [K, Nr, L_eff]
    phase = np.exp(-1j * 2.0 * np.pi * f_off[:, None, None] * tau_use[None, :, :]).astype(np.complex64)

    # a_use -> [1, Nr, L_eff] then multiply and sum over L_eff
    H = np.sum(phase * a_use[None, :, :], axis=-1)  # [K, Nr]
    return H.astype(np.complex64, copy=False)


def cfr_from_cir_batch(a_nml: np.ndarray,
                       tau_nml: np.ndarray,
                       f_off: np.ndarray,
                       num_paths_eff: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Reconstruct CFR for a BATCH of samples.

    Inputs:
      a_nml   : [N, Nr, L] complex64
      tau_nml : [N, Nr, L] float32
      f_abs   : [K] float64
      num_paths_eff: [N] int32 optional

    Output:
      H_nkm : [N, K, Nr] complex64

    Note: This is faster but can be memory heavy because it builds [N,K,Nr,L] phase.
    Prefer iter_cfr(...) for training.
    """
    _assert(a_nml.ndim == 3, f"a_nml must be [N,Nr,L], got {a_nml.shape}")
    _assert(tau_nml.shape == a_nml.shape, "tau_nml must have same shape as a_nml")
    _assert(f_off.ndim == 1, "f_abs must be [K]")

    N, Nr, L = a_nml.shape
    K = f_off.shape[0]

    if num_paths_eff is None:
        # Vectorized full-L
        phase = np.exp(-1j * 2.0 * np.pi * f_off[None, :, None, None] * tau_nml[:, None, :, :]).astype(np.complex64)
        H = np.sum(phase * a_nml[:, None, :, :], axis=-1)  # [N,K,Nr]
        return H.astype(np.complex64, copy=False)

    # If num_paths_eff differs per sample, do per-sample loop (still batch interface)
    H_out = np.zeros((N, K, Nr), dtype=np.complex64)
    for n in range(N):
        H_out[n] = cfr_from_cir_single(a_nml[n], tau_nml[n], f_off, int(num_paths_eff[n]))
    return H_out


def iter_cfr(dataset: CIRDataset,
             batch_size: int = 1,
             start: int = 0,
             end: Optional[int] = None) -> Iterator[Dict[str, Any]]:
    """
    Generator that yields batches with on-the-fly CFR reconstruction.

    Yields dict:
      {
        "user_position": [B,3],
        "H": [B,K,Nr] complex64,
        "pilot_mask": [T,K] bool or None
      }

    This avoids storing CFR on disk and avoids huge memory spikes.
    """
    N = dataset.a_full.shape[0]
    end = N if end is None else min(int(end), N)
    start = max(0, int(start))
    _assert(batch_size > 0, "batch_size must be > 0")

    f_off = _subcarrier_offsets(dataset.K, dataset.scs).astype(np.float64)

    i = start
    while i < end:
        j = min(i + batch_size, end)
        a = dataset.a_full[i:j]      # [B,Nr,L]
        tau = dataset.tau_full[i:j]  # [B,Nr,L]
        if dataset.num_paths_eff is None:
            H = cfr_from_cir_batch(a, tau, f_off, None)  # [B,K,Nr]
        else:
            H = cfr_from_cir_batch(a, tau, f_off, dataset.num_paths_eff[i:j])

        yield {
            "user_position": dataset.user_position[i:j],
            "H": H,
            "pilot_mask": dataset.pilot_mask
        }
        i = j


# ----------------------------
# Quick self-test (optional)
# ----------------------------
if __name__ == "__main__":
    # Example usage:
    ds = load_cir_dataset("D:/Graduation_Project/dataset/nf_rt_dataset_28g_32x32_1x1_k1024_CIR_4.npz")
    f_off = _subcarrier_offsets(ds.K, ds.scs)
    print(f"USER_POSITION={ds.user_position[0]}")
    H0 = cfr_from_cir_single(ds.a_full[0], ds.tau_full[0], f_off, None if ds.num_paths_eff is None else int(ds.num_paths_eff[0]))

    print("H0 shape:", H0.shape)  # expected [K, Nr]
    print(f"H0_value={H0[0,0]})")
    print(f"H0_MAX={np.max(np.abs(H0))}")
    print(f"H0_RMS={np.sqrt(np.mean(np.abs(H0)**2))}")
    # Example usage:
    # ds = load_cir_dataset("/content/drive/MyDrive/Graduation_Project/dataset/nf_rt_dataset_28g_32x32_1x1_k1024_CIR.npz")
    # f_abs = subcarrier_frequencies_abs(ds.fc, ds.K, ds.scs)
    # H0 = cfr_from_cir_single(ds.a_full[0], ds.tau_full[0], f_abs, None if ds.num_paths_eff is None else int(ds.num_paths_eff[0]))
    # print("H0 shape:", H0.shape)  # expected [K, Nr]
    pass

#1. 检查-1是不是无效路径
#2. 把对应的tau和a置零
