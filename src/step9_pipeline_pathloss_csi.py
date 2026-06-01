#!/usr/bin/env python3
"""
Step 9: Blender → Sionna RT → Pathloss + CSI 完整管线

端到端管线：
  Blender 场景 (mitsuba XML 或 PLY 或程序化) → Sionna RT 加载
  → PathSolver 求解传播路径 → CIR 提取 → CSI 张量构建（CIR + CFR）
  → RadioMapSolver 无线电地图 → 3D 场景渲染 → 9 张可视化图 + .npz 数据

用法:
    cd pipeline
    python step9_pipeline_pathloss_csi.py                              # 默认: 程序化场景, 双频段
    python step9_pipeline_pathloss_csi.py --freq 28                    # 仅 28 GHz
    python step9_pipeline_pathloss_csi.py --ofdm                       # 启用 OFDM CFR 计算
    python step9_pipeline_pathloss_csi.py --scene-xml my_scene.xml     # Mitsuba 直出
    python step9_pipeline_pathloss_csi.py --ply-dir ./meshes --material-map ./materials.json
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from scipy.constants import c as SPEED_OF_LIGHT

import sionna
from sionna.rt import (
    load_scene, PlanarArray, Transmitter, Receiver,
    PathSolver, RadioMapSolver, Camera,
)

# 导入 step8 的场景接口
from step8_blender_scene_interface import (
    BlenderSceneSpec, validate_blender_export, load_scene_from_spec,
    scene_summary,
)

# ── 默认配置 ──────────────────────────────────────────────────────────
FREQ_35G = 3.5e9
FREQ_28G = 28e9
OUTPUT_DIR = Path(__file__).resolve().parent.parent / "pipeline"
FIG_DIR = OUTPUT_DIR / "figures"
DATA_DIR = OUTPUT_DIR / "data"


# ── 管线核心 ──────────────────────────────────────────────────────────

def run_pipeline_for_freq(spec: BlenderSceneSpec, freq_label: str, freq_hz: float,
                          cell_size: tuple, args, scene, tmpdir: str) -> dict:
    """
    针对单个频段执行完整管线。

    Returns
    -------
    dict
        keys: paths, radio_map, csi_cir, csi_cfr (if --ofdm), metrics
    """
    print(f"\n{'─' * 60}")
    print(f"  [{freq_label}] 频段 = {freq_hz/1e9:.1f} GHz")
    print(f"{'─' * 60}")

    scene.frequency = freq_hz
    wavelength = SPEED_OF_LIGHT / freq_hz
    print(f"  波长 = {wavelength * 1e3:.2f} mm")

    # ── Step 1: PathSolver ──
    print(f"\n  [1/6] PathSolver (max_depth={spec.max_depth})...")
    t0 = time.perf_counter()
    solver = PathSolver()
    paths = solver(
        scene,
        max_depth=spec.max_depth,
        los=True, specular_reflection=True,
        diffuse_reflection=False, diffraction=False,
        edge_diffraction=False, refraction=False,
        seed=42,
    )
    t_path = time.perf_counter() - t0
    n_paths = paths.a.shape[-1] if hasattr(paths.a, 'shape') else paths.a.shape[-1]
    print(f"  路径数 = {n_paths}, 耗时 = {t_path*1e3:.1f} ms")

    # ── Step 2: CIR 提取 + CSI 张量构建 ──
    print(f"\n  [2/6] CIR 提取 + CSI 张量构建...")
    t0 = time.perf_counter()

    # paths.cir() 返回 (a_cpx, tau)
    sampling_freq = 122.88e6
    a_cpx, tau = paths.cir(
        sampling_frequency=sampling_freq,
        num_time_steps=1,
        normalize_delays=False,
        out_type="numpy",
    )
    # a_cpx: [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps]
    # tau:   [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]
    print(f"  a_cpx shape: {a_cpx.shape}")
    print(f"  tau shape:   {tau.shape}")

    # 提取每个 TX-RX 对的路径幅度和时延
    num_tx = a_cpx.shape[2]
    num_paths = a_cpx.shape[4]

    # 简化：取 [0,0,tx_idx,0,:,0] (单天线 SISO)
    csi_cir = {
        "coefficients": a_cpx,       # complex64
        "delays": tau,               # float64 [s]
        "num_paths": num_paths,
        "num_tx": num_tx,
        "sampling_frequency": sampling_freq,
    }

    # 提取多普勒和角度信息
    if hasattr(paths, 'doppler'):
        doppler = paths.doppler.numpy() if hasattr(paths.doppler, 'numpy') else paths.doppler
        csi_cir["doppler"] = doppler
    if hasattr(paths, 'theta_t'):
        csi_cir["theta_t"] = paths.theta_t.numpy() if hasattr(paths.theta_t, 'numpy') else paths.theta_t
        csi_cir["phi_t"] = paths.phi_t.numpy() if hasattr(paths.phi_t, 'numpy') else paths.phi_t
        csi_cir["theta_r"] = paths.theta_r.numpy() if hasattr(paths.theta_r, 'numpy') else paths.theta_r
        csi_cir["phi_r"] = paths.phi_r.numpy() if hasattr(paths.phi_r, 'numpy') else paths.phi_r

    t_csi = time.perf_counter() - t0
    print(f"  CIR 提取耗时 = {t_csi*1e3:.1f} ms")

    # ── Step 3: OFDM CFR 重建（可选）──
    csi_cfr = None
    if args.ofdm:
        print(f"\n  [3/6] OFDM CFR 重建 (K={args.num_subcarriers})...")
        t0 = time.perf_counter()
        K = args.num_subcarriers
        scs = args.bandwidth / K
        f_offsets = (np.arange(K, dtype=np.float64) - K // 2) * scs  # [K] Hz

        # H[tx_idx, rx_idx, k] = sum_p a_p * exp(-j*2pi*f_k*tau_p)
        H_cfr = np.zeros((num_tx, 1, K), dtype=np.complex64)
        for tx_idx in range(num_tx):
            a_p = a_cpx[0, 0, tx_idx, 0, :, 0]   # [num_paths]
            tau_p = tau[0, 0, tx_idx, 0, :]        # [num_paths]
            # 跳过无效路径
            valid = np.abs(a_p) > 1e-12
            a_valid = a_p[valid]
            tau_valid = tau_p[valid]
            if len(a_valid) > 0:
                phase = np.exp(-1j * 2.0 * np.pi *
                               f_offsets[:, np.newaxis] * tau_valid[np.newaxis, :])
                H_cfr[tx_idx, 0, :] = np.sum(
                    phase * a_valid[np.newaxis, :], axis=-1
                ).astype(np.complex64)

        csi_cfr = {
            "H": H_cfr,                          # [num_tx, num_rx, K] complex64
            "K": K,
            "subcarrier_spacing": scs,
            "bandwidth": args.bandwidth,
            "f_offsets": f_offsets,
        }
        t_cfr = time.perf_counter() - t0
        print(f"  CFR shape: {H_cfr.shape}, 耗时 = {t_cfr*1e3:.1f} ms")
    else:
        print(f"\n  [3/6] OFDM CFR 跳过 (使用 --ofdm 启用)")

    # ── Step 4: RadioMapSolver ──
    print(f"\n  [4/6] RadioMapSolver (cell={cell_size}, samples={spec.samples_per_tx})...")
    t0 = time.perf_counter()
    rm_solver = RadioMapSolver()
    radio_map = rm_solver(
        scene,
        center=spec.map_center,
        orientation=[0, 0, 0],
        size=spec.map_size,
        cell_size=cell_size,
        samples_per_tx=spec.samples_per_tx,
        max_depth=spec.max_depth,
        los=True, specular_reflection=True,
        diffuse_reflection=False, diffraction=False,
        seed=42,
    )
    pg = radio_map.path_gain.numpy()   # [num_tx, cells_y, cells_x]
    pg_db = 10 * np.log10(pg + 1e-15)
    rss = radio_map.rss.numpy()
    t_rm = time.perf_counter() - t0
    print(f"  RadioMap shape: {pg.shape}, 耗时 = {t_rm:.1f} s")

    # ── Step 5: 3D 场景渲染 ──
    print(f"\n  [5/6] 3D 场景渲染...")
    t0 = time.perf_counter()

    # 为渲染设置合适的相机
    scene.remove("camera")  # 先清理已有相机
    camera = Camera(
        name="camera",
        position=[30.0, 30.0, 25.0],
        look_at=spec.map_center,
    )
    scene.add(camera)

    # 渲染 1: pathloss 叠加到 3D 场景
    render_pathloss_path = str(FIG_DIR / f"pipeline_scene_pathloss_{freq_label.replace('.', 'p')}.png")
    try:
        scene.render_to_file(
            camera=camera,
            filename=render_pathloss_path,
            num_samples=256,
            resolution=[1200, 800],
            radio_map=radio_map,
            rm_metric="path_gain",
            rm_db_scale=True,
            rm_vmin=-160, rm_vmax=-60,
            show_devices=True,
        )
        print(f"  pathloss 场景渲染: {render_pathloss_path}")
    except Exception as e:
        print(f"  [WARN] pathloss 场景渲染失败: {e}")
        # 回退：用 matplotlib 做一个简单的场景概况图
        _render_fallback_scene_overview(spec, radio_map, freq_label, pg_db)

    # 渲染 2: 射线路径可视化
    render_rays_path = str(FIG_DIR / f"pipeline_scene_rays_{freq_label.replace('.', 'p')}.png")
    try:
        scene.render_to_file(
            camera=camera,
            filename=render_rays_path,
            num_samples=256,
            resolution=[1200, 800],
            paths=paths,
            show_devices=True,
        )
        print(f"  射线路径渲染: {render_rays_path}")
    except Exception as e:
        print(f"  [WARN] 射线路径渲染失败: {e}")

    t_render = time.perf_counter() - t0
    print(f"  渲染耗时 = {t_render:.1f} s")

    # ── Step 6: 导出指标 ──
    print(f"\n  [6/6] 导出信道指标...")
    metrics = _compute_metrics(a_cpx, tau, num_tx, freq_hz)
    _print_metrics(metrics, freq_label)

    # ── 生成 matplotlib 图 ──
    _generate_pathloss_figure(spec, pg_db, rss, freq_label, freq_hz)
    _generate_csi_magnitude_figure(a_cpx, tau, csi_cfr, freq_label, freq_hz)
    _generate_csi_phase_figure(a_cpx, tau, csi_cfr, freq_label, freq_hz)
    _generate_delay_spread_figure(a_cpx, tau, metrics, freq_label, freq_hz)
    _generate_angular_spread_figure(csi_cir, freq_label, freq_hz)
    _generate_summary_4panel(spec, pg_db, a_cpx, tau, csi_cir, metrics, freq_label, freq_hz)

    # ── 保存数据 ──
    npz_path = DATA_DIR / f"pipeline_csi_{freq_label.replace('.', 'p')}.npz"
    np.savez_compressed(
        str(npz_path),
        a_cpx=a_cpx,
        tau=tau,
        path_gain=pg,
        path_gain_db=pg_db,
        rss=rss,
        **{k: v for k, v in metrics.items() if isinstance(v, (np.ndarray, float, int))},
        freq_hz=freq_hz,
    )
    print(f"  CSI 数据已保存: {npz_path}")

    return {
        "paths": paths, "radio_map": radio_map,
        "csi_cir": csi_cir, "csi_cfr": csi_cfr,
        "metrics": metrics, "pg_db": pg_db,
    }


# ── 指标计算 ──────────────────────────────────────────────────────────

def _compute_metrics(a_cpx, tau, num_tx, freq_hz):
    """计算所有信道指标"""
    metrics = {}
    wavelength = SPEED_OF_LIGHT / freq_hz

    for tx_idx in range(num_tx):
        a_p = a_cpx[0, 0, tx_idx, 0, :, 0]
        tau_p = tau[0, 0, tx_idx, 0, :]
        valid = np.abs(a_p) > 1e-12
        a_v = a_p[valid]
        tau_v = tau_p[valid]

        if len(a_v) == 0:
            continue

        power = np.abs(a_v) ** 2
        total_power = np.sum(power)

        # Per-path pathloss
        pl_per_path = -20 * np.log10(np.abs(a_v) + 1e-15)

        # Combined pathloss
        pl_combined = -10 * np.log10(total_power + 1e-15)

        # Free-space pathloss (for reference, using first arriving path delay)
        min_tau_idx = np.argmin(tau_v)
        d_min = tau_v[min_tau_idx] * SPEED_OF_LIGHT
        pl_free_space = 20 * np.log10(4 * np.pi * max(d_min, 1e-6) / wavelength)

        # RMS delay spread
        mean_tau = np.sum(power * tau_v) / total_power
        rms_ds = np.sqrt(np.sum(power * tau_v**2) / total_power - mean_tau**2)

        # Path sorting by power
        sorted_idx = np.argsort(power)[::-1]
        power_sorted = power[sorted_idx]

        metrics[f"tx{tx_idx}"] = {
            "pl_per_path_db": pl_per_path,
            "pl_combined_db": pl_combined,
            "pl_free_space_db": pl_free_space,
            "rms_delay_spread_s": rms_ds,
            "rms_delay_spread_ns": rms_ds * 1e9,
            "total_power": total_power,
            "mean_tau_s": mean_tau,
            "power_per_path": power_sorted,
            "n_valid_paths": len(a_v),
        }

    return metrics


def _print_metrics(metrics, freq_label):
    """打印指标摘要"""
    for tx_key, m in metrics.items():
        print(f"  [{freq_label}] {tx_key}:")
        print(f"    PL combined = {m['pl_combined_db']:.1f} dB")
        print(f"    PL free-space (ref) = {m['pl_free_space_db']:.1f} dB")
        print(f"    RMS delay spread = {m['rms_delay_spread_ns']:.2f} ns")
        print(f"    Valid paths = {m['n_valid_paths']}")


# ── 图 1: Pathloss 热力图 ─────────────────────────────────────────────

def _generate_pathloss_figure(spec, pg_db, rss, freq_label, freq_hz):
    num_tx = pg_db.shape[0]
    map_size = spec.map_size

    fig, axes = plt.subplots(2, num_tx + 1, figsize=(5 * (num_tx + 1), 10),
                             constrained_layout=True)

    all_valid = pg_db[pg_db > -200]
    vmin = np.percentile(all_valid, 2) if len(all_valid) > 0 else -160
    vmax = np.percentile(all_valid, 98) if len(all_valid) > 0 else -60

    # Row 0: Path Gain per-BS + combined
    for i in range(num_tx):
        ax = axes[0, i]
        im = ax.imshow(pg_db[i], origin="lower", cmap="jet",
                       vmin=vmin, vmax=vmax,
                       extent=[-map_size[0]/2, map_size[0]/2,
                               -map_size[1]/2, map_size[1]/2])
        tx_name = spec.bs_positions[i][0]
        ax.set_title(f"BS {i}: {tx_name}")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.plot(spec.drone_position[0], spec.drone_position[1],
                "wx", markersize=10, markeredgewidth=2)
        bs_pos = spec.bs_positions[i][1]
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)

    ax = axes[0, num_tx]
    combined = pg_db.max(axis=0)
    im = ax.imshow(combined, origin="lower", cmap="jet", vmin=vmin, vmax=vmax,
                   extent=[-map_size[0]/2, map_size[0]/2,
                           -map_size[1]/2, map_size[1]/2])
    ax.set_title("Combined (max over BS)"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.plot(spec.drone_position[0], spec.drone_position[1],
            "wx", markersize=10, markeredgewidth=2)
    for _, bs_pos in spec.bs_positions:
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)
    plt.colorbar(im, ax=axes[0, :].tolist(), label="Path Gain [dB]", shrink=0.6)

    # Row 1: RSS
    for i in range(num_tx):
        ax = axes[1, i]
        im = ax.imshow(rss[i], origin="lower", cmap="jet",
                       extent=[-map_size[0]/2, map_size[0]/2,
                               -map_size[1]/2, map_size[1]/2])
        ax.set_title(f"RSS BS {i}: {spec.bs_positions[i][0]}")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.plot(spec.drone_position[0], spec.drone_position[1],
                "wx", markersize=10, markeredgewidth=2)
        bs_pos = spec.bs_positions[i][1]
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)

    ax = axes[1, num_tx]
    rss_combined = rss.max(axis=0)
    im = ax.imshow(rss_combined, origin="lower", cmap="jet",
                   extent=[-map_size[0]/2, map_size[0]/2,
                           -map_size[1]/2, map_size[1]/2])
    ax.set_title("RSS Combined (max over BS)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.plot(spec.drone_position[0], spec.drone_position[1],
            "wx", markersize=10, markeredgewidth=2)
    for _, bs_pos in spec.bs_positions:
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)
    plt.colorbar(im, ax=axes[1, :].tolist(), label="RSS [dBm]", shrink=0.6)

    fig.suptitle(f"Pathloss Maps — {freq_label} ({freq_hz/1e9:.1f} GHz)\n"
                 f"Drone @ {spec.drone_position}, {num_tx} Base Stations",
                 fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / f"pipeline_pathloss_{freq_label.replace('.', 'p')}.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [图] pathloss 热力图已保存")


# ── 图 2: CSI 幅度 ────────────────────────────────────────────────────

def _generate_csi_magnitude_figure(a_cpx, tau, csi_cfr, freq_label, freq_hz):
    num_tx = a_cpx.shape[2]
    ncols = 3 if csi_cfr is not None else 2
    fig, axes = plt.subplots(1, ncols, figsize=(6 * ncols, 5), constrained_layout=True)

    # Panel 1: per-path |a|^2 bar chart (first TX)
    ax = axes[0]
    a_p = a_cpx[0, 0, 0, 0, :, 0]
    power = np.abs(a_p) ** 2
    valid = power > 1e-12
    power_v = power[valid]
    sorted_power = np.sort(power_v)[::-1]
    ax.bar(range(len(sorted_power)), 10 * np.log10(sorted_power + 1e-15),
           color="steelblue", edgecolor="navy", alpha=0.8)
    ax.set_xlabel("Path Index (sorted by power)")
    ax.set_ylabel("Power [dB]")
    ax.set_title(f"Per-Path Power Distribution (BS 0)\n{len(sorted_power)} valid paths")
    ax.grid(axis="y", alpha=0.3)

    # Panel 2: CFR magnitude |H[k]| across subcarriers (if OFDM)
    if csi_cfr is not None:
        ax = axes[1]
        H = csi_cfr["H"]  # [num_tx, num_rx, K]
        K = csi_cfr["K"]
        freqs_mhz = csi_cfr["f_offsets"] / 1e6
        for tx_idx in range(min(num_tx, 3)):
            H_mag = np.abs(H[tx_idx, 0, :])
            ax.plot(freqs_mhz, H_mag, linewidth=0.8,
                    label=f"BS {tx_idx}")
        ax.set_xlabel("Frequency Offset [MHz]")
        ax.set_ylabel("|H(f)|")
        ax.set_title(f"CFR Magnitude (K={K})")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)

        ax_panel3 = axes[2]
    else:
        ax_panel3 = axes[1]

    # Panel 3: path delay vs power scatter
    for tx_idx in range(min(num_tx, 3)):
        a_p = a_cpx[0, 0, tx_idx, 0, :, 0]
        tau_p = tau[0, 0, tx_idx, 0, :]
        valid = np.abs(a_p) > 1e-12
        ax_panel3.scatter(tau_p[valid] * 1e9, 10 * np.log10(np.abs(a_p[valid])**2 + 1e-15),
                          alpha=0.6, s=30, label=f"BS {tx_idx}")
    ax_panel3.set_xlabel("Delay [ns]")
    ax_panel3.set_ylabel("Power [dB]")
    ax_panel3.set_title("Power-Delay Scatter")
    ax_panel3.legend(fontsize=8)
    ax_panel3.grid(alpha=0.3)

    fig.suptitle(f"CSI Magnitude — {freq_label} ({freq_hz/1e9:.1f} GHz)",
                 fontsize=13, fontweight="bold")
    fig.savefig(FIG_DIR / f"pipeline_csi_magnitude_{freq_label.replace('.', 'p')}.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [图] CSI 幅度图已保存")


# ── 图 3: CSI 相位 ────────────────────────────────────────────────────

def _generate_csi_phase_figure(a_cpx, tau, csi_cfr, freq_label, freq_hz):
    ncols = 2 if csi_cfr is not None else 1
    fig, axes = plt.subplots(1, ncols, figsize=(6 * ncols, 5), constrained_layout=True)
    if ncols == 1:
        axes = [axes]

    # Panel 1: per-path phase vs delay
    ax = axes[0]
    for tx_idx in range(min(a_cpx.shape[2], 3)):
        a_p = a_cpx[0, 0, tx_idx, 0, :, 0]
        tau_p = tau[0, 0, tx_idx, 0, :]
        valid = np.abs(a_p) > 1e-12
        phases = np.angle(a_p[valid], deg=True)
        ax.scatter(tau_p[valid] * 1e9, phases, alpha=0.6, s=30,
                   label=f"BS {tx_idx}")
    ax.set_xlabel("Delay [ns]")
    ax.set_ylabel("Phase [deg]")
    ax.set_title("Per-Path Phase vs Delay")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_ylim(-185, 185)

    # Panel 2: CFR phase (if OFDM)
    if csi_cfr is not None:
        ax = axes[1]
        H = csi_cfr["H"]
        K = csi_cfr["K"]
        freqs_mhz = csi_cfr["f_offsets"] / 1e6
        for tx_idx in range(min(a_cpx.shape[2], 3)):
            phase = np.angle(H[tx_idx, 0, :], deg=True)
            ax.plot(freqs_mhz, phase, linewidth=0.8, label=f"BS {tx_idx}")
        ax.set_xlabel("Frequency Offset [MHz]")
        ax.set_ylabel("Phase [deg]")
        ax.set_title(f"CFR Phase (K={K})")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
        ax.set_ylim(-185, 185)

    fig.suptitle(f"CSI Phase — {freq_label} ({freq_hz/1e9:.1f} GHz)",
                 fontsize=13, fontweight="bold")
    fig.savefig(FIG_DIR / f"pipeline_csi_phase_{freq_label.replace('.', 'p')}.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [图] CSI 相位图已保存")


# ── 图 4: Delay Spread ────────────────────────────────────────────────

def _generate_delay_spread_figure(a_cpx, tau, metrics, freq_label, freq_hz):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)

    # Panel 1: Power Delay Profile (first TX)
    ax = axes[0]
    a_p = a_cpx[0, 0, 0, 0, :, 0]
    tau_p = tau[0, 0, 0, 0, :]
    valid = np.abs(a_p) > 1e-12
    power = np.abs(a_p[valid]) ** 2
    tau_v = tau_p[valid] * 1e9  # ns
    ax.stem(tau_v, 10 * np.log10(power + 1e-15), basefmt=" ",
            linefmt="steelblue", markerfmt="o")
    rms_ds = metrics.get("tx0", {}).get("rms_delay_spread_ns", 0)
    ax.axvline(x=rms_ds, color="red", linestyle="--", linewidth=2,
               label=f"RMS-DS = {rms_ds:.2f} ns")
    ax.set_xlabel("Delay [ns]")
    ax.set_ylabel("Power [dB]")
    ax.set_title(f"Power Delay Profile (BS 0)")
    ax.legend()
    ax.grid(alpha=0.3)

    # Panel 2: RMS delay spread per TX
    ax = axes[1]
    tx_labels = []
    rms_values = []
    for tx_key, m in metrics.items():
        tx_labels.append(tx_key)
        rms_values.append(m["rms_delay_spread_ns"])
    ax.bar(tx_labels, rms_values, color="steelblue", edgecolor="navy")
    ax.set_xlabel("TX")
    ax.set_ylabel("RMS Delay Spread [ns]")
    ax.set_title("RMS Delay Spread per Base Station")
    for i, v in enumerate(rms_values):
        ax.text(i, v + 0.1, f"{v:.2f}", ha="center", fontsize=9)
    ax.grid(axis="y", alpha=0.3)

    fig.suptitle(f"Delay Spread Analysis — {freq_label} ({freq_hz/1e9:.1f} GHz)",
                 fontsize=13, fontweight="bold")
    fig.savefig(FIG_DIR / f"pipeline_delay_spread_{freq_label.replace('.', 'p')}.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [图] Delay spread 图已保存")


# ── 图 5: Angular Spread ──────────────────────────────────────────────

def _generate_angular_spread_figure(csi_cir, freq_label, freq_hz):
    fig, axes = plt.subplots(2, 2, figsize=(11, 10),
                             subplot_kw={"projection": "polar"},
                             constrained_layout=True)

    # 尝试提取角度信息
    has_angles = all(k in csi_cir for k in ["theta_t", "phi_t", "theta_r", "phi_r"])

    if not has_angles:
        for ax in axes.flat:
            ax.text(0.5, 0.5, "Angular data not available\n(use synthetic_array=True)",
                    ha="center", va="center", transform=ax.transAxes, fontsize=11)
            ax.set_title("No Data", fontsize=12)
    else:
        # 取第一个 TX-RX 对
        theta_t = np.array(csi_cir["theta_t"]).flatten()
        phi_t = np.array(csi_cir["phi_t"]).flatten()
        theta_r = np.array(csi_cir["theta_r"]).flatten()
        phi_r = np.array(csi_cir["phi_r"]).flatten()

        titles = [
            (f"AoD Zenith (θ_t)\n{freq_label}", theta_t, phi_t),
            (f"AoD Azimuth (φ_t)\n{freq_label}", phi_t, theta_t),
            (f"AoA Zenith (θ_r)\n{freq_label}", theta_r, phi_r),
            (f"AoA Azimuth (φ_r)\n{freq_label}", phi_r, theta_r),
        ]
        for ax, (title, r_vals, th_vals) in zip(axes.flat, titles):
            if len(r_vals) > 0:
                sizes = 50 * np.ones_like(r_vals)
                ax.scatter(th_vals, np.degrees(r_vals), s=sizes, alpha=0.7, c="steelblue")
            ax.set_title(title, fontsize=11, pad=15)

    fig.suptitle(f"Angular Spread — {freq_label} ({freq_hz/1e9:.1f} GHz)",
                 fontsize=13, fontweight="bold")
    fig.savefig(FIG_DIR / f"pipeline_angular_spread_{freq_label.replace('.', 'p')}.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [图] Angular spread 图已保存")


# ── 图 6: 四合一总览 ──────────────────────────────────────────────────

def _generate_summary_4panel(spec, pg_db, a_cpx, tau, csi_cir, metrics, freq_label, freq_hz):
    fig = plt.figure(figsize=(14, 12), constrained_layout=True)
    gs = GridSpec(2, 2, figure=fig)

    # (a) Pathloss heatmap (combined)
    ax = fig.add_subplot(gs[0, 0])
    combined = pg_db.max(axis=0)
    im = ax.imshow(combined, origin="lower", cmap="jet",
                   extent=[-spec.map_size[0]/2, spec.map_size[0]/2,
                           -spec.map_size[1]/2, spec.map_size[1]/2])
    ax.set_title("(a) Combined Path Gain [dB]"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.plot(spec.drone_position[0], spec.drone_position[1],
            "wx", markersize=10, markeredgewidth=2)
    for _, bs_pos in spec.bs_positions:
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=10, markeredgewidth=2)
    plt.colorbar(im, ax=ax, shrink=0.85)

    # (b) Power Delay Profile
    ax = fig.add_subplot(gs[0, 1])
    a_p = a_cpx[0, 0, 0, 0, :, 0]
    tau_p = tau[0, 0, 0, 0, :]
    valid = np.abs(a_p) > 1e-12
    ax.stem(tau_p[valid] * 1e9, 10 * np.log10(np.abs(a_p[valid])**2 + 1e-15),
            basefmt=" ", linefmt="steelblue", markerfmt="o")
    rms_ds = metrics.get("tx0", {}).get("rms_delay_spread_ns", 0)
    pl = metrics.get("tx0", {}).get("pl_combined_db", 0)
    ax.set_title(f"(b) Power Delay Profile (BS 0)\nRMS-DS = {rms_ds:.2f} ns")
    ax.set_xlabel("Delay [ns]"); ax.set_ylabel("Power [dB]")
    ax.grid(alpha=0.3)

    # (c) CSI magnitude (per-path bar)
    ax = fig.add_subplot(gs[1, 0])
    power = np.sort(np.abs(a_p[valid]) ** 2)[::-1]
    ax.bar(range(len(power)), 10 * np.log10(power + 1e-15),
           color="steelblue", edgecolor="navy", alpha=0.8)
    ax.set_title(f"(c) CSI Magnitude (BS 0)\nPL = {pl:.1f} dB")
    ax.set_xlabel("Path Index"); ax.set_ylabel("Power [dB]")
    ax.grid(axis="y", alpha=0.3)

    # (d) Angular spread (AoA polar)
    ax = fig.add_subplot(gs[1, 1], projection="polar")
    has_angles = all(k in csi_cir for k in ["theta_r", "phi_r"])
    if has_angles:
        theta_r = np.array(csi_cir["theta_r"]).flatten()
        phi_r = np.array(csi_cir["phi_r"]).flatten()
        if len(theta_r) > 0:
            ax.scatter(phi_r, np.degrees(theta_r), s=60, alpha=0.7, c="steelblue")
    ax.set_title(f"(d) AoA Distribution", pad=15)

    fig.suptitle(f"Pipeline Summary — {freq_label} ({freq_hz/1e9:.1f} GHz)\n"
                 f"Drone scene, {pg_db.shape[0]} BS, {len(power)} paths",
                 fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / f"pipeline_summary_4panel_{freq_label.replace('.', 'p')}.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [图] 四合一总览已保存")


# ── 双频对比图 ─────────────────────────────────────────────────────────

def _generate_dual_band_comparison(results: dict, spec: BlenderSceneSpec):
    """3.5 GHz vs 28 GHz 双频对比"""
    if len(results) < 2:
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 12), constrained_layout=True)

    for col, (freq_label, data) in enumerate(results.items()):
        pg_db = data["pg_db"]
        combined = pg_db.max(axis=0)

        # Row 0: pathloss heatmap
        ax = axes[0, col]
        im = ax.imshow(combined, origin="lower", cmap="jet",
                       extent=[-spec.map_size[0]/2, spec.map_size[0]/2,
                               -spec.map_size[1]/2, spec.map_size[1]/2])
        ax.set_title(f"Combined Path Gain — {freq_label}")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.plot(spec.drone_position[0], spec.drone_position[1],
                "wx", markersize=10, markeredgewidth=2)
        for _, bs_pos in spec.bs_positions:
            ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=10, markeredgewidth=2)
        plt.colorbar(im, ax=ax, shrink=0.85)

        # Row 1: PDP
        ax = axes[1, col]
        a_cpx = data["csi_cir"]["coefficients"]
        tau = data["csi_cir"]["delays"]
        a_p = a_cpx[0, 0, 0, 0, :, 0]
        tau_p = tau[0, 0, 0, 0, :]
        valid = np.abs(a_p) > 1e-12
        ax.stem(tau_p[valid] * 1e9, 10 * np.log10(np.abs(a_p[valid])**2 + 1e-15),
                basefmt=" ", linefmt="steelblue", markerfmt="o")
        m = data["metrics"].get("tx0", {})
        rms_ds = m.get("rms_delay_spread_ns", 0)
        pl = m.get("pl_combined_db", 0)
        ax.set_title(f"PDP (BS 0) — {freq_label}\nPL = {pl:.1f} dB, RMS-DS = {rms_ds:.2f} ns")
        ax.set_xlabel("Delay [ns]"); ax.set_ylabel("Power [dB]")
        ax.grid(alpha=0.3)

    fig.suptitle("Dual-Band Comparison: 3.5 GHz vs 28 GHz\n"
                 f"Drone scene, {spec.max_depth} bounces",
                 fontsize=14, fontweight="bold")
    fig.savefig(FIG_DIR / "pipeline_dual_band_comparison.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [图] 双频对比图已保存")


# ── 场景渲染回退 ──────────────────────────────────────────────────────

def _render_fallback_scene_overview(spec, radio_map, freq_label, pg_db):
    """当 scene.render_to_file() 不可用时，用 matplotlib 渲染场景概况"""
    fig, ax = plt.subplots(figsize=(12, 9), constrained_layout=True)
    combined = pg_db.max(axis=0)
    im = ax.imshow(combined, origin="lower", cmap="jet",
                   extent=[-spec.map_size[0]/2, spec.map_size[0]/2,
                           -spec.map_size[1]/2, spec.map_size[1]/2])
    ax.set_title(f"Scene Pathloss Overview — {freq_label}\n"
                 f"(3D render fallback — matplotlib)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.plot(spec.drone_position[0], spec.drone_position[1],
            "wx", markersize=12, markeredgewidth=3, label="Drone RX")
    for name, pos in spec.bs_positions:
        ax.plot(pos[0], pos[1], "k^", markersize=14, markeredgewidth=2)
        ax.annotate(name, (pos[0] + 1, pos[1] + 1), fontsize=9, color="white",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="black", alpha=0.7))
    ax.legend()
    plt.colorbar(im, ax=ax, label="Path Gain [dB]", shrink=0.85)
    fig.savefig(FIG_DIR / f"pipeline_scene_pathloss_{freq_label.replace('.', 'p')}.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)


# ── 主流程 ────────────────────────────────────────────────────────────

def run(args):
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Step 9: Blender → Sionna RT → Pathloss + CSI 完整管线")
    print("=" * 60)

    # ── 构建场景规范 ──
    spec = BlenderSceneSpec(
        scene_xml_path=args.scene_xml,
        ply_dir=args.ply_dir,
        material_map_path=args.material_map,
        max_depth=args.max_depth,
        samples_per_tx=args.samples_per_tx,
    )

    # 验证
    issues = validate_blender_export(spec)
    for issue in issues:
        print(f"  {issue}")
    errors = [i for i in issues if i.startswith("[ERROR]")]
    if errors:
        print("  验证失败，退出")
        return

    # ── 加载场景 ──
    print("\n[1] 加载场景...")
    scene, tmpdir = load_scene_from_spec(spec)
    scene_summary(spec, scene)

    # ── 确定频段 ──
    freqs_to_run = []
    if args.freq in ("3.5", "all"):
        freqs_to_run.append(("3.5GHz", FREQ_35G, spec.cell_sizes["3.5GHz"]))
    if args.freq in ("28", "all"):
        freqs_to_run.append(("28GHz", FREQ_28G, spec.cell_sizes["28GHz"]))

    # ── 逐频段执行 ──
    results = {}
    for freq_label, freq_hz, cell_size in freqs_to_run:
        data = run_pipeline_for_freq(spec, freq_label, freq_hz, cell_size, args, scene, tmpdir)
        results[freq_label] = data

    # ── 双频对比 ──
    if len(results) == 2:
        print(f"\n[7] 双频对比...")
        _generate_dual_band_comparison(results, spec)

    # ── 清理 ──
    if tmpdir and os.path.exists(tmpdir):
        shutil.rmtree(tmpdir, ignore_errors=True)

    print("\n" + "=" * 60)
    print("管线完成!")
    print(f"  图: {FIG_DIR}")
    print(f"  数据: {DATA_DIR}")
    print(f"  共生成 {len(list(FIG_DIR.glob('pipeline_*.png')))} 张图")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Blender → Sionna RT → Pathloss + CSI 完整管线"
    )
    # 场景来源
    parser.add_argument("--scene-xml", default=None,
                        help="Mitsuba XML 场景路径（mitsuba-blender 直出）")
    parser.add_argument("--ply-dir", default=None,
                        help="PLY 网格目录（Blender PLY 导出）")
    parser.add_argument("--material-map", default=None,
                        help="材质映射 JSON（配合 --ply-dir 使用）")
    # 频段
    parser.add_argument("--freq", default="all", choices=["3.5", "28", "all"],
                        help="频段选择 (default: all)")
    # OFDM
    parser.add_argument("--ofdm", action="store_true",
                        help="启用 OFDM CFR 计算")
    parser.add_argument("--num-subcarriers", type=int, default=1024,
                        help="OFDM 子载波数 (default: 1024)")
    parser.add_argument("--bandwidth", type=float, default=122.88e6,
                        help="带宽 [Hz] (default: 122.88e6)")
    # RT 参数
    parser.add_argument("--max-depth", type=int, default=5,
                        help="射线追踪最大深度 (default: 5)")
    parser.add_argument("--samples-per-tx", type=int, default=5_000_000,
                        help="RadioMapSolver 每 TX 采样数 (default: 5000000)")
    args = parser.parse_args()
    run(args)
