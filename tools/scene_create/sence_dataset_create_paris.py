"""Generate a near-field RT dataset with per-antenna CIR snapshots."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

try:
    import sionna.rt
    from sionna.rt import (
        LambertianPattern,
        PathSolver,
        PlanarArray,
        Receiver,
        Transmitter,
        load_scene,
    )
except ImportError as exc:
    raise ImportError(
        "Missing dependency 'sionna-rt'. Activate the 'sionna' environment and "
        "install it before running this script."
    ) from exc

# 角度范围：与可视化代码一致，限制在 [0, pi)
THETA_MIN = 0.0
THETA_MAX = np.pi

FC = 28e9
SCS = 120e3
K = 1024
T = 14

BS_ROWS, BS_COLS = 32, 32
NR_ANT = BS_ROWS * BS_COLS

C0 = 3e8
FS = K * SCS

TP = [1, 3, 6, 8, 11, 13]
COMB = 8
K0 = 0

NUM_SAMPLES = 10000
R_MIN, R_MAX = 1.0, 10.0
UE_Z = 6
BS_POS = np.array([30.146, -72.924, 6.0], dtype=np.float32)
L_KEEP = 32

DEFAULT_OUT_PATH = (
    Path(__file__).resolve().parent / "nf_rt_dataset_28g_32x32_1x1_k1024_CIR_Paris_10000.npz"
)


def build_pilot_mask(
    t_sym: int,
    k_sc: int,
    tp_list: list[int],
    comb: int,
    k0: int,
) -> np.ndarray:
    mask = np.zeros((t_sym, k_sc), dtype=np.bool_)
    tp0 = [t - 1 for t in tp_list]
    kp = np.arange(k0, k_sc, comb, dtype=np.int32)
    for t in tp0:
        mask[t, kp] = True
    return mask


def sample_rx_position(
    center_pos: np.ndarray,
    r_min: float,
    r_max: float,
    theta_min: float,
    theta_max: float,
    z: float = UE_Z,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """
    Uniformly sample a UE position in an annular sector around the BS.

    半径按面积均匀采样：
        r^2 ~ U(r_min^2, r_max^2)

    角度按扇区均匀采样：
        theta ~ U(theta_min, theta_max)
    """
    if rng is None:
        rng = np.random.default_rng()

    # 面积均匀，而不是半径均匀
    u = rng.uniform(0.0, 1.0)
    r_squared = u * (r_max**2 - r_min**2) + r_min**2
    r = np.sqrt(r_squared)

    # 扇区角度均匀采样
    theta = rng.uniform(theta_min, theta_max)

    x = center_pos[0] + r * np.cos(theta)
    y = center_pos[1] + r * np.sin(theta)

    return np.array([x, y, z], dtype=np.float32)


def build_scene_and_solver(preview: bool = False) -> tuple[object, PathSolver]:
    scene = load_scene(sionna.rt.scene.etoile)
    scene.frequency = FC

    scene.tx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )
    scene.rx_array = PlanarArray(
        num_rows=BS_ROWS,
        num_cols=BS_COLS,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )

    scene.add(
        Receiver(
            name="bs",
            position=BS_POS,
            orientation=[0, 0, 0],
        )
    )
    scene.add(
        Transmitter(
            name="ue",
            position=np.array([BS_POS[0] + 1.0, BS_POS[1], UE_Z], dtype=np.float32),
            orientation=[0, 0, 0],
            power_dbm=0.0,
        )
    )

    for obj in scene.objects.values():
        if hasattr(obj, "radio_material") and obj.radio_material is not None:
            obj.radio_material.scattering_pattern = LambertianPattern()
            obj.radio_material.scattering_coefficient = 0.3

    if preview:
        scene.preview()

    return scene, PathSolver()


def generate_one_sample(
    scene,
    p_solver: PathSolver,
    rng: np.random.Generator | None = None,
) -> dict[str, np.ndarray]:
    if rng is None:
        rng = np.random.default_rng()

    ue_pos = sample_rx_position(
        center_pos=BS_POS,
        r_min=R_MIN,
        r_max=R_MAX,
        theta_min=THETA_MIN,
        theta_max=THETA_MAX,
        z=UE_Z,
        rng=rng,
    )
    scene.get("ue").position = ue_pos

    paths = p_solver(
        scene,
        max_depth=5,
        los=True,
        specular_reflection=True,
        diffuse_reflection=True,
        diffraction=False,
        edge_diffraction=False,
        refraction=False,
        synthetic_array=False,
    )

    # NumPy output avoids TensorFlow/DLPack device conversion failures.
    a, tau = paths.cir(
        sampling_frequency=FS,
        normalize_delays=False,
        out_type="numpy",
    )

    a0_np = np.asarray(a[0, :, 0, 0, :, 0], dtype=np.complex64)
    tau0_np = np.asarray(tau[0, :, 0, 0, :], dtype=np.float32)

    power = np.sum(np.abs(a0_np) ** 2, axis=0)
    l_eff = min(L_KEEP, int(power.shape[0]))
    idx = np.argsort(-power)[:l_eff].astype(np.int32)

    a_keep = np.zeros((NR_ANT, L_KEEP), dtype=np.complex64)
    tau_keep = np.zeros((NR_ANT, L_KEEP), dtype=np.float32)

    if l_eff > 0:
        a_keep[:, :l_eff] = a0_np[:, idx]
        tau_keep[:, :l_eff] = tau0_np[:, idx]

    return {
        "user_position": ue_pos.astype(np.float32),
        "a_full": a_keep,
        "tau_full": tau_keep,
        "num_paths_eff": np.int32(l_eff),
    }


def main(
    out_path: str | Path = DEFAULT_OUT_PATH,
    seed: int = 42,
    num_samples: int = NUM_SAMPLES,
    preview: bool = False,
) -> None:
    rng = np.random.default_rng(seed)
    out_path = Path(out_path)

    pilot_mask = build_pilot_mask(T, K, TP, COMB, K0)
    scene, p_solver = build_scene_and_solver(preview=preview)

    positions = np.zeros((num_samples, 3), dtype=np.float32)
    a_full_all = np.zeros((num_samples, NR_ANT, L_KEEP), dtype=np.complex64)
    tau_full_all = np.zeros((num_samples, NR_ANT, L_KEEP), dtype=np.float32)
    num_paths_eff_all = np.zeros((num_samples,), dtype=np.int32)

    for i in range(num_samples):
        sample = generate_one_sample(scene, p_solver, rng=rng)
        positions[i] = sample["user_position"]
        a_full_all[i] = sample["a_full"]
        tau_full_all[i] = sample["tau_full"]
        num_paths_eff_all[i] = sample["num_paths_eff"]
        print(f"[{i + 1}/{num_samples}] done")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out_path,
        user_position=positions,
        a_full=a_full_all,
        tau_full=tau_full_all,
        pilot_mask=pilot_mask,
        num_paths_eff=num_paths_eff_all,
        fc=np.float64(FC),
        scs=np.float64(SCS),
        K=np.int32(K),
        fs=np.float64(FS),
        L_keep=np.int32(L_KEEP),
        bs_pos=BS_POS.astype(np.float32),
    )
    print(f"Saved: {out_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT_PATH)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--num-samples", type=int, default=NUM_SAMPLES)
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Open the Sionna scene preview before dataset generation.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(
        out_path=args.out,
        seed=args.seed,
        num_samples=args.num_samples,
        preview=args.preview,
    )
