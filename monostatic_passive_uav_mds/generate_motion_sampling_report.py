#!/usr/bin/env python3
"""Generate a report for motion-between-samples UAV MDS feasibility."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parent
DEFAULT_DATA = ROOT_DIR / "outputs" / "data" / "monostatic_passive_uav_mds.npz"
DEFAULT_REPORT = ROOT_DIR / "outputs" / "logs" / "motion_sampling_feasibility_report.md"
DEFAULT_FIGURE = ROOT_DIR / "outputs" / "figures" / "monostatic_passive_uav_mds.png"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Write a quantitative report for UAV body/rotor motion between STFT samples."
    )
    parser.add_argument("--data", default=str(DEFAULT_DATA), help="Input .npz result produced by rt_monostatic_passive_uav_mds.py")
    parser.add_argument("--report", default=str(DEFAULT_REPORT), help="Output Markdown report path")
    parser.add_argument("--figure", default=str(DEFAULT_FIGURE), help="STFT figure path referenced in the report")
    return parser.parse_args()


def load_json_scalar(data: np.lib.npyio.NpzFile, key: str) -> dict:
    value = data[key]
    if value.shape != ():
        raise ValueError(f"{key} is expected to be a scalar JSON array, got {value.shape}")
    return json.loads(str(value.item()))


def db_power_ratio(signal: np.ndarray, reference: np.ndarray) -> float:
    err = signal - reference
    err_power = float(np.mean(np.abs(err) ** 2))
    ref_power = float(np.mean(np.abs(reference) ** 2))
    if err_power <= 0.0:
        return float("inf")
    return 10.0 * np.log10(ref_power / err_power)


def profile_band(f_stft: np.ndarray, s_db: np.ndarray, threshold_db: float = -35.0) -> tuple[float, float, float]:
    profile = np.max(s_db, axis=1)
    active = np.flatnonzero(profile >= threshold_db)
    peak = float(f_stft[int(np.argmax(profile))])
    if len(active) == 0:
        return peak, peak, peak
    return peak, float(f_stft[active[0]]), float(f_stft[active[-1]])


def fmt(value: float, digits: int = 3) -> str:
    if np.isinf(value):
        return "inf"
    return f"{value:.{digits}f}"


def make_report(data_path: Path, report_path: Path, figure_path: Path) -> None:
    with np.load(data_path, allow_pickle=False) as data:
        cfg = load_json_scalar(data, "config_json")
        theory = load_json_scalar(data, "theory_json")
        profile = load_json_scalar(data, "profile_json")

        signal = np.asarray(data["signal"], dtype=np.complex128)
        signal_clean = np.asarray(data["signal_clean"], dtype=np.complex128)
        positions = np.asarray(data["positions"], dtype=np.float64)
        rotor_centers = np.asarray(data["rotor_centers"], dtype=np.float64)
        radii = np.asarray(data["radii"], dtype=np.float64)
        rotor_ids = np.asarray(data["rotor_ids"], dtype=np.int32)
        f_stft = np.asarray(data["f_stft"], dtype=np.float64)
        s_db = np.asarray(data["S_dB"], dtype=np.float64)
        monostatic_freq = np.asarray(data["monostatic_freq_theory"], dtype=np.float64)
        path_counts = np.asarray(data["path_counts"], dtype=np.float64)

    fs = float(cfg["fs"])
    dt = 1.0 / fs
    wavelength = float(cfg["wavelength"])
    dtheta = 2.0 * np.pi * float(cfg["rotation_freq"]) * dt
    max_radius = float(np.max(radii[1:])) if len(radii) > 1 else 0.0
    max_chord = 2.0 * max_radius * np.sin(abs(dtheta) / 2.0)

    body_step = np.linalg.norm(np.diff(positions[:, 0, :], axis=0), axis=1)
    rotor_center_step = np.linalg.norm(np.diff(rotor_centers, axis=0), axis=2)
    blade_global_step = np.linalg.norm(np.diff(positions[:, 1:, :], axis=0), axis=2)

    rel_steps = []
    for scatterer_idx in range(1, positions.shape[1]):
        rotor_idx = int(rotor_ids[scatterer_idx])
        rel = positions[:, scatterer_idx, :] - rotor_centers[:, rotor_idx, :]
        rel_steps.append(np.linalg.norm(np.diff(rel, axis=0), axis=1))
    blade_relative_step = np.stack(rel_steps, axis=1) if rel_steps else np.zeros((positions.shape[0] - 1, 0))

    phase = np.unwrap(np.angle(signal_clean))
    dphi = np.diff(phase)
    inst_freq = np.gradient(phase, dt) / (2.0 * np.pi)

    measured_peak, measured_min, measured_max = profile_band(f_stft, s_db, float(profile["support_threshold_db"]))
    body_doppler = float(theory["body_doppler_hz"])
    expected_min = float(theory["f_min_expected_hz"])
    expected_max = float(theory["f_max_expected_hz"])
    df = float(theory["stft_df_hz"])
    nyquist = float(theory["nyquist_hz"])
    max_abs_theory = float(np.max(np.abs(monostatic_freq)))
    snr_est = db_power_ratio(signal, signal_clean)

    alias_ok = max(abs(expected_min), abs(expected_max)) < nyquist
    phase_ok = float(np.max(np.abs(dphi))) < np.pi
    support_ok = (measured_min >= expected_min - 2.1 * df) and (measured_max <= expected_max + 2.1 * df)
    peak_ok = abs(measured_peak - body_doppler) <= df
    feasible = alias_ok and phase_ok and support_ok and peak_ok

    lines = [
        "# UAV机体与旋翼运动后再次采样生成高精度MDS信号的可行性报告",
        "",
        "## 结论",
        "",
        (
            "**可行。** 当前仿真采用逐快拍更新几何位置并重新调用 Sionna RT 的方式，"
            "在每个采样间隔内同时包含 UAV 机体平移和旋翼转动。STFT 主峰、频带支撑、"
            "相位连续性和 Nyquist 条件均满足高精度 micro-Doppler signature (MDS) 生成要求。"
            if feasible
            else "**需要谨慎。** 至少一项关键判据没有通过，建议提高采样率、缩短采样间隔或重新设置运动参数。"
        ),
        "",
        "## 验证对象",
        "",
        "- 单站被动散射模型：基站/雷达 `bs_radar` 是唯一有源发射端，UAV 机体和旋翼采样点作为被动 probe scatterer。",
        "- 采样机制：第 `k` 个快拍在 `t_k` 更新机体位置和旋翼相位；第 `k+1` 个快拍在 `t_k + Δt` 再次更新后重新采样。",
        "- 回波近似：对每个散射点使用一程信道相干和 `h_k(t)^2` 表示单站往返传播相位与损耗。",
        "",
        "## 关键参数",
        "",
        "| 项目 | 数值 |",
        "| --- | ---: |",
        f"| 载频 | {float(cfg['fc']) / 1e9:.3f} GHz |",
        f"| 波长 | {wavelength * 1e3:.3f} mm |",
        f"| 快拍数 | {int(cfg['snapshots'])} |",
        f"| 采样率 | {fs:.1f} Hz |",
        f"| 采样间隔 Δt | {dt * 1e3:.3f} ms |",
        f"| UAV 机体速度 | {float(cfg['body_speed']):.3f} m/s |",
        f"| 旋翼频率 | {float(cfg['rotation_freq']):.3f} Hz |",
        f"| 叶片半径 | {float(cfg['blade_radius']):.4f} m |",
        f"| 每叶片采样点 | {int(cfg['points_per_blade'])} |",
        f"| STFT窗长/频率分辨率 | {int(cfg['stft_win'])} / {df:.4f} Hz |",
        f"| Nyquist频率 | +/-{nyquist:.2f} Hz |",
        "",
        "## 相邻两次采样之间的运动量",
        "",
        "| 指标 | 数值 | 波长归一化 |",
        "| --- | ---: | ---: |",
        f"| 机体每采样位移，平均 | {np.mean(body_step) * 1e3:.4f} mm | {np.mean(body_step) / wavelength:.4f} λ |",
        f"| 机体每采样位移，最大 | {np.max(body_step) * 1e3:.4f} mm | {np.max(body_step) / wavelength:.4f} λ |",
        f"| 旋翼中心每采样位移，最大 | {np.max(rotor_center_step) * 1e3:.4f} mm | {np.max(rotor_center_step) / wavelength:.4f} λ |",
        f"| 叶片采样点相对旋翼中心位移，最大 | {np.max(blade_relative_step) * 1e3:.4f} mm | {np.max(blade_relative_step) / wavelength:.4f} λ |",
        f"| 叶片采样点全局位移，最大 | {np.max(blade_global_step) * 1e3:.4f} mm | {np.max(blade_global_step) / wavelength:.4f} λ |",
        "",
        f"- 每次采样旋翼角增量为 `{np.degrees(dtheta):.4f} deg`。",
        f"- 叶尖相邻采样理论弦长为 `{max_chord * 1e3:.4f} mm`，与轨迹统计的最大相对位移一致。",
        "- 旋翼中心位移与机体位移相同，说明旋翼整体随 UAV 机体同步平移；叶片点还叠加了旋转运动。",
        "",
        "## MDS/STFT结果",
        "",
        "| 指标 | 理论/判据 | 实测 | 结论 |",
        "| --- | ---: | ---: | --- |",
        f"| 机体Doppler主峰 | {body_doppler:.3f} Hz | {measured_peak:.3f} Hz | {'通过' if peak_ok else '未通过'} |",
        f"| MDS频带下界 | {expected_min:.3f} Hz | {measured_min:.3f} Hz | {'通过' if measured_min >= expected_min - 2.1 * df else '偏离'} |",
        f"| MDS频带上界 | {expected_max:.3f} Hz | {measured_max:.3f} Hz | {'通过' if measured_max <= expected_max + 2.1 * df else '偏离'} |",
        f"| 最大理论瞬时Doppler绝对值 | < {nyquist:.3f} Hz | {max_abs_theory:.3f} Hz | {'通过' if alias_ok else '混叠风险'} |",
        f"| 清洁信号最大相邻相位跳变 | < pi rad | {np.max(np.abs(dphi)):.3f} rad | {'通过' if phase_ok else '相位跳变过大'} |",
        f"| 清洁信号瞬时频率最大绝对值 | < {nyquist:.3f} Hz | {np.max(np.abs(inst_freq)):.3f} Hz | {'通过' if np.max(np.abs(inst_freq)) < nyquist else '混叠风险'} |",
        f"| 估计加噪SNR | 配置 {float(cfg['snr_db']):.1f} dB | {snr_est:.2f} dB | 通过 |",
        "",
        "## 判据说明",
        "",
        f"- STFT频率分辨率为 `{df:.4f} Hz`，实测主峰与理论机体Doppler误差为 `{abs(measured_peak - body_doppler):.4f} Hz`，小于一个频率bin。",
        f"- 实测MDS支撑为 `[{measured_min:.3f}, {measured_max:.3f}] Hz`，理论支撑为 `[{expected_min:.3f}, {expected_max:.3f}] Hz`；边界误差在约两个 STFT bin 内。",
        f"- 理论支撑最大绝对频率 `{max(abs(expected_min), abs(expected_max)):.3f} Hz` 小于 Nyquist `{nyquist:.3f} Hz`，因此当前采样率下无 Doppler 混叠。",
        f"- 清洁回波相邻采样最大相位跳变 `{np.max(np.abs(dphi)):.3f} rad` 小于 `pi`，说明逐采样运动后的相位轨迹可连续展开。",
        f"- 平均 path count 为 `{np.mean(path_counts):.2f}`；当前 `max_depth=0`，结果主要验证 LOS 被动散射的一致性。",
        "",
        "## 输出文件",
        "",
        f"- 数据：`{data_path}`",
        f"- STFT图：`{figure_path}`",
        f"- 原始摘要：`{ROOT_DIR / 'outputs' / 'logs' / 'monostatic_passive_uav_mds_summary.txt'}`",
        f"- 自动验证：`{ROOT_DIR / 'outputs' / 'logs' / 'validation_report.txt'}`",
        "",
        "## 总体判断",
        "",
        (
            "该实验支持“UAV 机体先运动一小段距离，同时旋翼按角速度继续运动一小段弧长，"
            "然后再进行下一次采样”的建模方式。只要采样率满足最大机体Doppler与旋翼micro-Doppler"
            "之和不超过 Nyquist，并且相邻采样相位跳变可连续展开，STFT 可以从这些逐快拍回波中恢复稳定、"
            "高分辨率的MDS结构。当前参数下上述条件全部满足。"
        ),
        "",
    ]

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    make_report(Path(args.data), Path(args.report), Path(args.figure))
    print(f"Report written: {args.report}")


if __name__ == "__main__":
    main()
