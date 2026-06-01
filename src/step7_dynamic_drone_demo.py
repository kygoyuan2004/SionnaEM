#!/usr/bin/env python3
"""
Step 7: 动态无人机演示 — 实时更新的路径损耗和信道图

两种模式:
  Mode A (逐步RT): 每步更新无人机位置 → 重新射线追踪 → 重新计算无线电地图
  Mode B (快速CIR调制): 一次性RT → CIR后处理叠加微多普勒 → 实时谱图

用法:
    cd src
    python step7_dynamic_drone_demo.py                      # 默认: Mode A + Mode B, 28 GHz
    python step7_dynamic_drone_demo.py --mode fast          # 仅 Mode B (快速)
    python step7_dynamic_drone_demo.py --mode full          # 仅 Mode A (全RT)
    python step7_dynamic_drone_demo.py --freq 3.5           # 3.5 GHz
    python step7_dynamic_drone_demo.py --steps 10            # 10步
"""

import argparse
import os
import sys
import tempfile
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# 加载项目模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from micro_doppler_modulator import UAVMicroDopplerConfig, MicroDopplerModulator

import sionna
from sionna.rt import (
    load_scene, PlanarArray, Transmitter, Receiver,
    PathSolver, RadioMapSolver,
)

# 复用 step6 的场景生成函数
from step6_static_drone_scene import (
    generate_drone_scene_xml, create_base_stations,
    FREQ_35G, FREQ_28G, BS_POSITIONS, DRONE_POS,
    MAP_CENTER, MAP_SIZE, RT_MAX_DEPTH, RT_SAMPLES_PER_TX,
)

# ── 动态仿真参数 ──────────────────────────────────────────────────

DRONE_VELOCITY = [2.0, 1.0, 0.0]        # 无人机水平速度 [m/s]
DRONE_VERTICAL_VEL = 0.0                 # 垂直速度 [m/s]
DT = 0.5                                 # 地图更新间隔 [s]
N_STEPS_DEFAULT = 10                     # 默认步数

# 微多普勒参数
FC_MOD = 28e9
F_ROT = 100.0                            # 旋翼旋转频率 [Hz]
R_BLADE = 0.15                           # 叶片半径 [m]
FS_SIGNAL = 50e3                         # 信号采样率 [Hz]
T_OBS = 0.2                              # 观测时长 [s]


# ── Mode A: 逐步RT ──────────────────────────────────────────────

def run_mode_a_full_rt(args):
    """
    Mode A: 逐步位移 → 重新RT → 重新RadioMap
    精确但是慢。适合生成关键帧序列。
    """
    output_dir = args.output_dir or "../figures"
    os.makedirs(output_dir, exist_ok=True)

    freq_hz = FREQ_35G if args.freq == "3.5" else FREQ_28G
    freq_label = f"{freq_hz/1e9:.1f}GHz"
    n_steps = args.steps or N_STEPS_DEFAULT
    cell_size = (2.0, 2.0) if args.freq == "3.5" else (1.0, 1.0)

    drone_pos = np.array(DRONE_POS, dtype=np.float64)
    velocity = np.array(DRONE_VELOCITY, dtype=np.float64)

    print("=" * 60)
    print(f"Step 7 Mode A: 逐步RT动态 — {freq_label}")
    print(f"  速度: {velocity} m/s, 步长: {DT}s, 步数: {n_steps}")
    print("=" * 60)

    with tempfile.TemporaryDirectory() as tmpdir:
        ply_dir = os.path.join(tmpdir, "meshes")
        xml_path = os.path.join(tmpdir, "drone_scene.xml")

        # 生成初始场景
        print("\n[1] 生成初始无人机场景...")
        xml_content = generate_drone_scene_xml(ply_dir, drone_pos.tolist())
        with open(xml_path, "w") as f:
            f.write(xml_content)

        scene = load_scene(xml_path)
        scene.tx_array = PlanarArray(
            num_rows=1, num_cols=1, vertical_spacing=0.5, horizontal_spacing=0.5,
            pattern="iso", polarization="V",
        )
        scene.rx_array = PlanarArray(
            num_rows=1, num_cols=1, vertical_spacing=0.5, horizontal_spacing=0.5,
            pattern="iso", polarization="V",
        )
        scene.frequency = freq_hz

        create_base_stations(scene, power_dbm=44)
        rx = Receiver(name="drone_rx", position=drone_pos.tolist(),
                      velocity=velocity.tolist(), orientation=[0, 0, 0])
        scene.add(rx)

        print(f"  初始位置: {drone_pos}")

        # 动态循环
        solver = PathSolver()
        rm_solver = RadioMapSolver()
        trajectory = [drone_pos.copy()]
        path_gains_over_time = []

        print(f"\n[2] 动态循环 ({n_steps} 步)...")
        t_start = time.time()

        for step in range(n_steps):
            # 更新位置
            drone_pos += velocity * DT
            trajectory.append(drone_pos.copy())

            # 更新RX位置
            scene.get("drone_rx").position = drone_pos.tolist()

            # 重新RT
            paths = solver(scene, max_depth=RT_MAX_DEPTH, los=True,
                           specular_reflection=True, diffuse_reflection=False,
                           diffraction=False, refraction=False, seed=42 + step)

            # 重新RadioMap（使用较少采样以加速）
            radio_map = rm_solver(
                scene,
                center=MAP_CENTER,
                size=MAP_SIZE,
                cell_size=cell_size,
                samples_per_tx=min(RT_SAMPLES_PER_TX, 3_000_000),
                max_depth=RT_MAX_DEPTH,
                los=True, specular_reflection=True,
                diffuse_reflection=False, diffraction=False,
                seed=42 + step,
            )

            pg = radio_map.path_gain.numpy()
            pg_db = 10 * np.log10(pg.max(axis=0) + 1e-15)
            path_gains_over_time.append(pg.max(axis=0).copy())

            t_elapsed = time.time() - t_start
            print(f"  Step {step + 1}/{n_steps}: pos=({drone_pos[0]:.1f}, "
                  f"{drone_pos[1]:.1f}, {drone_pos[2]:.1f}), "
                  f"path_gain max={pg_db.max():.1f} dB, "
                  f"{t_elapsed:.1f}s elapsed")

        t_total = time.time() - t_start
        trajectory = np.array(trajectory)

        print(f"\n  总耗时: {t_total:.1f}s, 平均 {t_total/n_steps:.1f}s/步")

        # ── 可视化 ──
        print("\n[3] 生成可视化...")
        fig, axes = plt.subplots(2, 3, figsize=(18, 12), constrained_layout=True)

        # 第一行: 初始和最终的信道图 + 轨迹
        for col_idx, (step_label, step_idx) in enumerate([("Initial", 0), ("Final", -1)]):
            ax = axes[0, col_idx]
            pg = path_gains_over_time[step_idx]
            im = ax.imshow(pg, origin="lower", cmap="jet",
                           extent=[-MAP_SIZE[0]/2, MAP_SIZE[0]/2,
                                   -MAP_SIZE[1]/2, MAP_SIZE[1]/2])
            ax.set_title(f"Path Gain — {step_label} (step {step_idx + 1})")
            ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
            ax.plot(trajectory[step_idx, 0], trajectory[step_idx, 1],
                    "wx", markersize=12, markeredgewidth=2)
            for _, bs_pos in BS_POSITIONS[:3]:
                ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=10)
            plt.colorbar(im, ax=ax, label="Path Gain [dB]")

        # 轨迹图
        ax = axes[0, 2]
        ax.plot(trajectory[:, 0], trajectory[:, 1], "b-o", markersize=4)
        ax.plot(trajectory[0, 0], trajectory[0, 1], "go", markersize=10, label="Start")
        ax.plot(trajectory[-1, 0], trajectory[-1, 1], "ro", markersize=10, label="End")
        for name, bs_pos in BS_POSITIONS[:3]:
            ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=10)
            ax.annotate(name, (bs_pos[0] + 1, bs_pos[1] + 1), fontsize=9)
        ax.set_title("Drone Trajectory")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.legend(); ax.set_aspect("equal")
        ax.set_xlim(-MAP_SIZE[0]/2, MAP_SIZE[0]/2)
        ax.set_ylim(-MAP_SIZE[1]/2, MAP_SIZE[1]/2)

        # 第二行: 路径增益随距离变化的分析
        for i in range(min(3, len(BS_POSITIONS))):
            ax = axes[1, i]
            bs_pos = np.array(BS_POSITIONS[i][1])
            distances = np.linalg.norm(trajectory - bs_pos, axis=1)
            # 从每个 path_gain 中提取在无人机位置附近的增益
            gains_at_drone = []
            for pg_map in path_gains_over_time:
                # path_gain shape: [cells_y, cells_x]
                # 找到无人机所在格点
                drone_x_idx = int((trajectory[len(gains_at_drone), 0] + MAP_SIZE[0]/2) / cell_size[0])
                drone_y_idx = int((trajectory[len(gains_at_drone), 1] + MAP_SIZE[1]/2) / cell_size[1])
                drone_x_idx = np.clip(drone_x_idx, 0, pg_map.shape[1] - 1)
                drone_y_idx = np.clip(drone_y_idx, 0, pg_map.shape[0] - 1)
                gains_at_drone.append(pg_map[drone_y_idx, drone_x_idx])
            gains_at_drone = np.array(gains_at_drone)

            ax.plot(distances, gains_at_drone, "o-", markersize=4)
            ax.set_xlabel(f"Distance to {BS_POSITIONS[i][0]} [m]")
            ax.set_ylabel("Path Gain [dB]")
            ax.set_title(f"Path Gain vs Distance — {BS_POSITIONS[i][0]}")
            ax.grid(True, alpha=0.3)

        fig.suptitle(f"Dynamic Drone — Mode A (Full RT) — {freq_label}\n"
                     f"Velocity: {velocity} m/s, {n_steps} steps × {DT}s, "
                     f"Total: {t_total:.1f}s",
                     fontsize=14, fontweight="bold")

        fig_path = os.path.join(output_dir,
                                f"step7_dynamic_modeA_{freq_label.replace('.', 'p')}.png")
        fig.savefig(fig_path, dpi=150, bbox_inches="tight")
        print(f"  保存: {fig_path}")

        return trajectory, path_gains_over_time, t_total


# ── Mode B: 快速CIR调制 ─────────────────────────────────────────

def run_mode_b_fast_cir(args):
    """
    Mode B: 基于 Doppler 的快速通道模拟
    一次性RT → CIR后处理叠加微多普勒 → 实时谱图
    利用项目现有的 MicroDopplerModulator
    """
    output_dir = args.output_dir or "../figures"
    os.makedirs(output_dir, exist_ok=True)

    freq_hz = FREQ_35G if args.freq == "3.5" else FREQ_28G
    freq_label = f"{freq_hz/1e9:.1f}GHz"

    print("=" * 60)
    print(f"Step 7 Mode B: 快速CIR调制 — {freq_label}")
    print(f"  旋翼: {F_ROT} Hz, 叶片半径: {R_BLADE}m, 平移速度: {DRONE_VELOCITY} m/s")
    print("=" * 60)

    # 参数
    fs_signal = FS_SIGNAL
    t_obs = T_OBS
    n_samples = int(fs_signal * t_obs)
    t_vec = np.arange(n_samples) / fs_signal

    # 微多普勒配置
    config = UAVMicroDopplerConfig(
        carrier_freq=freq_hz,
        blade_radius=R_BLADE,
        rotation_freq=F_ROT,
        n_rotors=4,
        n_blades_per_rotor=2,
        rotor_arm_length=0.175,
        body_amplitude=1.0,
        blade_amplitude=0.12,
        radar_position=BS_POSITIONS[0][1],  # 用第一个基站位置作为参考
        body_position=DRONE_POS,
        body_velocity=DRONE_VELOCITY,
        sampling_rate=fs_signal,
        obs_duration=t_obs,
    )

    # 纯解析微多普勒信号（不使用RT — 快速演示微多普勒特征）
    print("\n[1] 生成解析微多普勒信号...")
    modulator = MicroDopplerModulator(config)

    t_start = time.time()
    signal_complex = modulator.generate_received_signal(t_vec)
    t_gen = time.time() - t_start

    # STFT 谱图
    print("[2] 计算STFT谱图...")
    f_stft, t_stft, S_dB = modulator.stft_spectrogram(signal_complex)
    harmonics = modulator.detect_harmonics(signal_complex)

    print(f"  检测到 {len(harmonics)} 次谐波 (间距 = {F_ROT} Hz)")
    print(f"  信号生成: {t_gen*1e3:.1f}ms, 样本: {n_samples}")

    # ── 可视化 ──
    print("[3] 生成可视化...")
    fig, axes = plt.subplots(2, 2, figsize=(16, 12), constrained_layout=True)

    # 左上: 微多普勒谱图
    ax = axes[0, 0]
    f_khz = f_stft / 1e3
    freq_mask = np.abs(f_stft) < 20e3
    im = ax.pcolormesh(t_stft * 1e3, f_stft[freq_mask] / 1e3,
                       S_dB[freq_mask, :], shading="auto", cmap="jet")
    ax.set_xlabel("Time [ms]")
    ax.set_ylabel("Doppler Frequency [kHz]")
    ax.set_title(f"Micro-Doppler Spectrogram — {freq_label}, {F_ROT} Hz rotors")
    plt.colorbar(im, ax=ax, label="PSD [dB]")

    # 右上: 多普勒频率轮廓
    ax = axes[0, 1]
    doppler_profile = np.max(S_dB, axis=1)
    ax.plot(f_stft / 1e3, doppler_profile, "b-", linewidth=0.8)
    ax.set_xlabel("Doppler Frequency [kHz]")
    ax.set_ylabel("Max PSD [dB]")
    ax.set_title("Doppler Frequency Profile")
    ax.set_xlim(-20, 20)
    # 标注谐波
    for h in harmonics[:10]:
        f_h = h * F_ROT
        ax.axvline(f_h / 1e3, color="r", alpha=0.3, linestyle="--", linewidth=0.5)
        ax.axvline(-f_h / 1e3, color="r", alpha=0.3, linestyle="--", linewidth=0.5)
    ax.grid(True, alpha=0.3)

    # 左下: 瞬时频率 (短时间窗)
    ax = axes[1, 0]
    t_zoom = t_vec[:2000]
    sig_zoom = signal_complex[:2000]
    phase = np.unwrap(np.angle(sig_zoom))
    inst_freq = np.diff(phase) / (2 * np.pi) * fs_signal
    ax.plot(t_zoom[1:] * 1e3, inst_freq / 1e3, "b-", linewidth=0.5)
    ax.set_xlabel("Time [ms]")
    ax.set_ylabel("Instantaneous Frequency [kHz]")
    ax.set_title("Instantaneous Frequency (zoom)")
    ax.grid(True, alpha=0.3)

    # 右下: 信息面板
    ax = axes[1, 1]
    ax.axis("off")
    beta = config.beta_max
    f_dev_peak = config.f_dev_peak
    body_doppler = (-2 * freq_hz * np.linalg.norm(DRONE_VELOCITY)
                    / 3e8)  # 单基地往返
    info_lines = [
        f"Mode B: Fast CIR Modulation",
        f"",
        f"Frequency: {freq_hz/1e9:.1f} GHz",
        f"Wavelength: {3e8/freq_hz*1e3:.2f} mm",
        f"",
        f"Rotor frequency: {F_ROT} Hz ({F_ROT*60:.0f} RPM)",
        f"Blade radius: {R_BLADE}m",
        f"Number of rotors: {config.n_rotors}",
        f"Number of blades: {config.n_blades}",
        f"",
        f"Modulation index β: {beta:.1f} rad",
        f"Peak freq deviation: ±{f_dev_peak/1e3:.1f} kHz",
        f"Body Doppler shift: {body_doppler:.1f} Hz",
        f"",
        f"Detected harmonics: {len(harmonics)}",
        f"Harmonic spacing: {F_ROT} Hz",
        f"",
        f"Signal generation: {t_gen*1e3:.1f} ms",
    ]
    for i, line in enumerate(info_lines):
        ax.text(0.1, 0.95 - i * 0.045, line, transform=ax.transAxes,
                fontfamily="monospace", fontsize=10, verticalalignment="top")

    fig.suptitle(f"Dynamic Drone — Mode B (Fast CIR Modulation) — {freq_label}\n"
                 f"Analytic Micro-Doppler Signal, {n_samples} samples, "
                 f"{t_obs*1e3:.0f}ms observation",
                 fontsize=14, fontweight="bold")

    fig_path = os.path.join(output_dir,
                            f"step7_dynamic_modeB_{freq_label.replace('.', 'p')}.png")
    fig.savefig(fig_path, dpi=150, bbox_inches="tight")
    print(f"  保存: {fig_path}")

    # 如果两个频段都要计算，生成对比
    return signal_complex, f_stft, t_stft, S_dB


def run_both_frequencies_fast(args):
    """在两个频段上都运行 Mode B 并生成对比图"""
    output_dir = args.output_dir or "../figures"
    os.makedirs(output_dir, exist_ok=True)

    results = {}
    for freq_hz, label in [(FREQ_35G, "3.5GHz"), (FREQ_28G, "28GHz")]:
        print(f"\n--- {label} ---")
        config = UAVMicroDopplerConfig(
            carrier_freq=freq_hz,
            blade_radius=R_BLADE,
            rotation_freq=F_ROT,
            n_rotors=4, n_blades_per_rotor=2,
            rotor_arm_length=0.175,
            body_amplitude=1.0, blade_amplitude=0.12,
            radar_position=BS_POSITIONS[0][1],
            body_position=DRONE_POS,
            body_velocity=DRONE_VELOCITY,
            sampling_rate=FS_SIGNAL,
            obs_duration=T_OBS,
        )
        modulator = MicroDopplerModulator(config)
        t_vec = np.arange(int(FS_SIGNAL * T_OBS)) / FS_SIGNAL
        signal_complex = modulator.generate_received_signal(t_vec)
        f_stft, t_stft, S_dB = modulator.stft_spectrogram(signal_complex)
        results[label] = {
            "signal": signal_complex, "f_stft": f_stft,
            "t_stft": t_stft, "S_dB": S_dB,
            "beta": config.beta_max,
            "f_dev": config.f_dev_peak,
        }

    # 对比图
    fig, axes = plt.subplots(1, 3, figsize=(20, 6), constrained_layout=True)

    for idx, (label, data) in enumerate(results.items()):
        ax = axes[idx]
        f_stft = data["f_stft"]
        S_dB = data["S_dB"]
        freq_limit = 5000 if label == "3.5GHz" else 20000
        freq_mask = np.abs(f_stft) < freq_limit
        im = ax.pcolormesh(data["t_stft"] * 1e3, f_stft[freq_mask] / 1e3,
                           S_dB[freq_mask, :], shading="auto", cmap="jet")
        ax.set_xlabel("Time [ms]")
        ax.set_ylabel("Doppler Frequency [kHz]")
        ax.set_title(f"{label}\nβ={data['beta']:.1f} rad, "
                     f"f_dev=±{data['f_dev']/1e3:.1f} kHz")
        plt.colorbar(im, ax=ax, label="PSD [dB]")

    # 第三列: 多普勒剖面对比
    ax = axes[2]
    for label, data in results.items():
        doppler_profile = np.max(data["S_dB"], axis=1)
        ax.plot(data["f_stft"] / 1e3, doppler_profile, linewidth=0.8,
                label=f"{label} (β={data['beta']:.0f} rad)")
    ax.set_xlabel("Doppler Frequency [kHz]")
    ax.set_ylabel("Max PSD [dB]")
    ax.set_title("Doppler Profile Comparison")
    ax.set_xlim(-20, 20)
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle("Dual-Band Micro-Doppler Comparison\n"
                 f"4-rotor UAV, {F_ROT} Hz rotors, R={R_BLADE}m, "
                 f"v={DRONE_VELOCITY} m/s",
                 fontsize=14, fontweight="bold")

    fig_path = os.path.join(output_dir, "step7_dual_band_microdoppler_comparison.png")
    fig.savefig(fig_path, dpi=150, bbox_inches="tight")
    print(f"\n保存双频对比: {fig_path}")

    return results


def run(args):
    """主入口"""
    if args.mode in ("full", "all"):
        print("\n" + "=" * 60)
        print("Mode A: 逐步RT动态")
        print("=" * 60)
        try:
            run_mode_a_full_rt(args)
        except Exception as e:
            print(f"Mode A 出错: {e}")
            print("(Mode A 需要 GPU 支持 Sionna RT。尝试 Mode B...)")

    if args.mode in ("fast", "all"):
        print("\n" + "=" * 60)
        print("Mode B: 快速CIR调制")
        print("=" * 60)
        try:
            if args.freq == "all":
                run_both_frequencies_fast(args)
            else:
                run_mode_b_fast_cir(args)
        except Exception as e:
            print(f"Mode B 出错: {e}")

    print("\n" + "=" * 60)
    print("Step 7 完成!")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Step 7: 动态无人机演示")
    parser.add_argument("--mode", default="all", choices=["full", "fast", "all"],
                        help="full=逐步RT, fast=快速CIR调制, all=两种 (default: all)")
    parser.add_argument("--freq", default="28", choices=["3.5", "28", "all"],
                        help="频段选择 (Mode B 可用 all) (default: 28)")
    parser.add_argument("--steps", type=int, default=None,
                        help="Mode A 步数 (default: 10)")
    parser.add_argument("--output-dir", default=None,
                        help="输出目录 (default: ../figures)")
    args = parser.parse_args()
    run(args)
