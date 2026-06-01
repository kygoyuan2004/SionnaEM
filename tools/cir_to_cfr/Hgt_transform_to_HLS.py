# -*- coding: utf-8 -*-
"""
Hgt_transform_to_HLS.py

Online (on-the-fly) generation:
  CIR(sample) -> CFR H_km [K,Nr] -> pilot obs + AWGN -> LS on pilots -> HLS

No saving to NPZ. Designed to be imported in model_input/train.

Assumptions (aligned with your current dataset):
- K=1024, T=14
- Nr = 32x32 = 1024 (BS antennas)
- Quasi-static over one slot: use one CFR snapshot for all OFDM symbols
- Pilot pattern: TP=[1,3,6,8,11,13], COMB=8, K0=0
"""

from __future__ import annotations
import numpy as np
from typing import Optional, Dict, Any, Tuple
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
# ------------------------------------------------------------
# Import your CIR->CFR utilities
# ------------------------------------------------------------
try:
    from dataset_CIR_to_CFR import load_cir_dataset, cfr_from_cir_single, _subcarrier_offsets
except Exception as e:
    try:
        from cir_to_cfr.dataset_CIR_to_CFR import load_cir_dataset, cfr_from_cir_single, _subcarrier_offsets
    except Exception:
        raise ImportError(
            "Cannot import dataset_CIR_to_CFR. Put it in the same folder or add to PYTHONPATH. "
            f"Original error: {repr(e)}"
        )

# ------------------------------------------------------------
# Defaults (you can override in HLSOnTheFly init)
# ------------------------------------------------------------
T_DEFAULT = 14
K_DEFAULT = 1024
TP_DEFAULT = [1, 3, 6, 8, 11, 13]  # 1-indexed
COMB_DEFAULT = 8
K0_DEFAULT = 0

PILOT_SYMBOL_DEFAULT = 1.0 + 0.0j

# numerical
EPS_VALID = 1e-12

def _assert(cond: bool, msg: str):
    if not cond:
        raise ValueError(msg)

def build_pilot_mask(T: int, K: int, TP_1idx, comb: int, k0: int) -> np.ndarray:
    mask = np.zeros((T, K), dtype=np.bool_)
    tp0 = [t - 1 for t in TP_1idx]
    kp = np.arange(k0, K, comb, dtype=np.int32)
    for t in tp0:
        _assert(0 <= t < T, f"TP contains out-of-range symbol index: {t+1}")
        mask[t, kp] = True
    return mask

def pilot_tk_lists(T: int, K: int, TP_1idx, comb: int, k0: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Fixed pilot ordering:
      for t in TP ascending:
        for k in kp ascending:
          append (t,k)
    """
    pilot_mask = build_pilot_mask(T, K, TP_1idx, comb, k0)
    tp0 = np.array([t - 1 for t in TP_1idx], dtype=np.int32)
    kp = np.arange(k0, K, comb, dtype=np.int32)

    pilot_t = np.repeat(tp0, repeats=kp.size).astype(np.int16)
    pilot_k = np.tile(kp, reps=tp0.size).astype(np.int16)
    return pilot_mask, pilot_t, pilot_k

def per_sample_noise_var_from_Hp(Hp: np.ndarray, snr_db: float, pilot_symbol: complex) -> float:
    """
    Define SNR at pilot REs as:
      SNR = E|H*X|^2 / noise_var  => noise_var = E|H*X|^2 / 10^(SNR/10)

    Hp: [Npilots, Nr]
    """
    Xp = np.complex64(pilot_symbol)
    sig_pow = float(np.mean(np.abs(Hp * Xp) ** 2))
    sig_pow = max(sig_pow, 1e-20)
    noise_var = sig_pow / (10.0 ** (float(snr_db) / 10.0))
    return float(noise_var)

def complex_awgn(shape, noise_var: float, rng: np.random.Generator) -> np.ndarray:
    """CN(0, noise_var)"""
    sigma = np.sqrt(noise_var / 2.0)
    w = sigma * (rng.standard_normal(shape) + 1j * rng.standard_normal(shape))
    return w.astype(np.complex64)

def ls_from_Hkm_quasi_static(
    H_km: np.ndarray,         # [K, Nr]
    pilot_t: np.ndarray,      # [Npilots] int16
    pilot_k: np.ndarray,      # [Npilots] int16
    snr_db: float,
    rng: np.random.Generator,
    pilot_symbol: complex,
) -> Tuple[np.ndarray, float]:
    """
    Quasi-static: H(t,k,m) == H(k,m).
    Pilots-only LS:
      Hp = H_km[pilot_k]
      Yp = Hp*Xp + W
      HLS_pilots = Yp / Xp
    Returns:
      HLS_pilots: [Npilots, Nr]
      noise_var: float
    """
    _assert(H_km.ndim == 2, f"H_km must be [K,Nr], got {H_km.shape}")
    K, Nr = H_km.shape

    kk = pilot_k.astype(np.int32)
    Hp = H_km[kk, :]  # [Npilots, Nr]

    noise_var = per_sample_noise_var_from_Hp(Hp, snr_db, pilot_symbol)
    W = complex_awgn(Hp.shape, noise_var, rng)

    Xp = np.complex64(pilot_symbol)
    Yp = (Hp * Xp + W).astype(np.complex64)
    HLS = (Yp / Xp).astype(np.complex64)
    return HLS, noise_var
"""
采用 准静态（quasi-static）假设：一个样本的 H_km 在 T=14 个 OFDM 符号里不变，
"""
def pilots_to_sparse_grid(
    HLS_pilots: np.ndarray,   # [Npilots, Nr]
    pilot_t: np.ndarray,
    pilot_k: np.ndarray,
    T: int,
    K: int,
) -> np.ndarray:
    """Place pilots-only into [T,K,Nr], others 0."""
    _assert(HLS_pilots.ndim == 2, f"HLS_pilots must be [Npilots,Nr], got {HLS_pilots.shape}")
    Npilots, Nr = HLS_pilots.shape
    grid = np.zeros((T, K, Nr), dtype=np.complex64)
    grid[pilot_t.astype(np.int32), pilot_k.astype(np.int32), :] = HLS_pilots
    return grid

def nn_interp_from_pilots_one(
    HLS_pilots: np.ndarray,   # [Npilots, Nr] in pilot_tk_lists order
    T: int,
    K: int,
    TP_1idx,
    comb: int,
    k0: int,
) -> np.ndarray:
    """
    Fast nearest-neighbor interpolation to full [T,K,Nr].
    Works with your regular comb pilots.
    """
    tp0 = np.array([t - 1 for t in TP_1idx], dtype=np.int32)
    kp = np.arange(k0, K, comb, dtype=np.int32)
    Nk = kp.size
    Npilots, Nr = HLS_pilots.shape
    _assert(Npilots == tp0.size * Nk, f"Pilot count mismatch: {Npilots} vs {tp0.size*Nk}")

    # reshape to [Ntp, Nk, Nr]
    H_sym = HLS_pilots.reshape(tp0.size, Nk, Nr)

    # freq NN
    k_idx = np.arange(K, dtype=np.int32)
    q = np.rint((k_idx - k0) / float(comb)).astype(np.int32)
    q = np.clip(q, 0, Nk - 1)
    H_freq = H_sym[:, q, :]  # [Ntp, K, Nr]

    # time NN
    nearest_tp_pos = np.zeros((T,), dtype=np.int32)
    for t in range(T):
        nearest_tp_pos[t] = int(np.argmin(np.abs(tp0 - t)))

    return H_freq[nearest_tp_pos, :, :].astype(np.complex64, copy=False)

def complex_to_2ch(x: np.ndarray) -> np.ndarray:
    """
    Convert complex array to 2-channel real:
      [...,] complex -> [..., 2] float32  (last dim = [Re, Im])
    """
    return np.stack([x.real, x.imag], axis=-1).astype(np.float32, copy=False)

class HLSOnTheFly:
    """
    Load CIR dataset once, then for each idx:
      CIR[idx] -> CFR H_km -> LS pilots -> optional grid/interp
    """

    def __init__(
        self,
        cir_npz_path: str,
        snr_db: float = 0,
        seed: int = 42,
        pilot_symbol: complex = PILOT_SYMBOL_DEFAULT,
        T: int = T_DEFAULT,
        K: int = K_DEFAULT,
        TP_1idx=TP_DEFAULT,
        comb: int = COMB_DEFAULT,
        k0: int = K0_DEFAULT,
    ):
        self.ds = load_cir_dataset(cir_npz_path)
        _assert(self.ds.K == K, f"K mismatch: ds.K={self.ds.K}, expected K={K}")
        self.T = int(T)
        self.K = int(K)
        self.N = int(self.ds.a_full.shape[0])
        self.Nr = int(self.ds.a_full.shape[1])
        self.snr_db = float(snr_db)
        self.seed = int(seed)
        self.pilot_symbol = pilot_symbol

        self.TP_1idx = list(TP_1idx)
        self.comb = int(comb)
        self.k0 = int(k0)

        self.pilot_mask, self.pilot_t, self.pilot_k = pilot_tk_lists(self.T, self.K, self.TP_1idx, self.comb, self.k0)
        self.Npilots = int(self.pilot_t.size)

        # frequency offsets for CFR reconstruction
        self.f_off = _subcarrier_offsets(self.ds.K, self.ds.scs).astype(np.float64)

    def __len__(self) -> int:
        return self.N

    def get(
        self,
        idx: int,
        return_grid: str = "pilots",
        as_2ch: bool = False,
        return_gt: bool = True,
    ) -> Dict[str, Any]:
        """
        return_grid:
          - "pilots": return HLS_pilots only
          - "sparse": additionally return HLS_grid [T,K,Nr] sparse (nonpilots=0)
          - "nn":     additionally return HLS_grid [T,K,Nr] NN interpolated

        as_2ch:
          if True, convert complex arrays to float32 with last dim 2 (Re,Im)
        return_gt:
          if True, return H_km (GT CFR) [K,Nr]
        """
        idx = int(idx)
        _assert(0 <= idx < self.N, f"idx out of range: {idx} (N={self.N})")

        # deterministic per-sample RNG (important for reproducible training)
        rng = np.random.default_rng(self.seed + idx * 1000003)

        # CIR -> CFR
        L_eff = None if self.ds.num_paths_eff is None else int(self.ds.num_paths_eff[idx])
        H_km = cfr_from_cir_single(self.ds.a_full[idx], self.ds.tau_full[idx], self.f_off, L_eff)  # [K,Nr]

        # CFR -> LS pilots
        HLS_pilots, noise_var = ls_from_Hkm_quasi_static(
            H_km=H_km,
            pilot_t=self.pilot_t,
            pilot_k=self.pilot_k,
            snr_db=self.snr_db,
            rng=rng,
            pilot_symbol=self.pilot_symbol,
        )

        out: Dict[str, Any] = {
            "user_position": self.ds.user_position[idx].astype(np.float32, copy=False),
            "pilot_t": self.pilot_t,  # [Npilots]
            "pilot_k": self.pilot_k,  # [Npilots]
            "pilot_mask": self.pilot_mask,
            "noise_var": np.float32(noise_var),
        }

        if return_gt:
            out["H_km"] = H_km.astype(np.complex64, copy=False)

        out["HLS_pilots"] = HLS_pilots.astype(np.complex64, copy=False)

        if return_grid == "sparse":
            grid = pilots_to_sparse_grid(HLS_pilots, self.pilot_t, self.pilot_k, self.T, self.K)
            out["HLS_grid"] = grid
        elif return_grid == "nn":
            grid = nn_interp_from_pilots_one(HLS_pilots, self.T, self.K, self.TP_1idx, self.comb, self.k0)
            out["HLS_grid"] = grid
        elif return_grid == "pilots":
            pass
        else:
            raise ValueError(f"return_grid must be one of ['pilots','sparse','nn'], got {return_grid}")

        if as_2ch:
            if "H_km" in out:
                out["H_km_2ch"] = complex_to_2ch(out["H_km"])  # [K,Nr,2]
                del out["H_km"]
            out["HLS_pilots_2ch"] = complex_to_2ch(out["HLS_pilots"])  # [Npilots,Nr,2]
            del out["HLS_pilots"]
            if "HLS_grid" in out:
                out["HLS_grid_2ch"] = complex_to_2ch(out["HLS_grid"])  # [T,K,Nr,2]
                del out["HLS_grid"]

        return out


# quick sanity test
if __name__ == "__main__":
    engine = HLSOnTheFly(
        cir_npz_path=str(ROOT / "src" / "nf_rt_dataset_28g_32x32_1x1_k1024_CIR_5.npz"),
        snr_db=0,
        seed=42,
    )
    s = engine.get(0, return_grid="sparse", return_gt=True)
    H_km = s["H_km"]
    HLS = s["HLS_pilots"]
    HLS_s = s["HLS_grid"]
    s = engine.get(0, return_grid="nn", return_gt=True)
    HLS_nn = s["HLS_grid"]    
    print("USER_POSITION:", s["user_position"])
    print("H_km shape:", H_km.shape, "max|H|:", float(np.max(np.abs(H_km))), "rms|H|:", float(np.sqrt(np.mean(np.abs(H_km)**2))))
    print("HLS_pilots shape:", HLS.shape, "max|HLS|:", float(np.max(np.abs(HLS))), "noise_var:", float(s["noise_var"]))
    print("pilot count:", int(s["pilot_t"].size), "Nr:", engine.Nr)
    print(f"HLS's Vlaue is {H_km[0,0]}")
    print(f"H_sphase's value is {HLS_s[0,0,0]}")
    print(f"H_sphase's value is {HLS_nn[0,0,0]}")
