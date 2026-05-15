#!/usr/bin/env python3
"""SionnaRT 环境验证脚本 —— 逐项检查各组件是否正确安装并能正常工作."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

# ── 辅助 ────────────────────────────────────────────────────────────────
PASS = 0
FAIL = 0


def check(name: str) -> None:
    global PASS
    PASS += 1
    print(f"  [PASS] ({PASS:02d}) {name}")


def fail(name: str, detail: str = "") -> None:
    global FAIL
    FAIL += 1
    print(f"  [FAIL] ({FAIL:02d}) {name}")
    if detail:
        print(f"         {detail}")


def section(title: str) -> None:
    print(f"\n{'='*68}")
    print(f"  {title}")
    print(f"{'='*68}")


# ── 1. 基础依赖 ─────────────────────────────────────────────────────────
section("1. Python & 基础依赖")

print(f"  Python: {sys.version}")

try:
    import numpy as np
    check(f"numpy {np.__version__}")
except Exception:
    fail("numpy import")

try:
    import tensorflow as tf
    check(f"tensorflow {tf.__version__}")
except Exception as e:
    fail("tensorflow import", str(e))
    sys.exit(1)

# ── 2. TensorFlow GPU ───────────────────────────────────────────────────
section("2. TensorFlow GPU 检测")

gpus = tf.config.list_physical_devices("GPU")
if gpus:
    for gpu in gpus:
        check(f"GPU found: {gpu.name}")
    # 获取 GPU 详细信息
    try:
        details = tf.config.experimental.get_device_details(gpus[0])
        desc = details.get("device_name", "unknown")
        check(f"GPU 详情: {desc}")
    except Exception:
        check("GPU 可用（无法获取详情）")
else:
    fail("No GPU found — 将使用 CPU 运行", "检查 CUDA/cuDNN 安装")

# ── 3. Sionna 核心包 ────────────────────────────────────────────────────
section("3. Sionna 核心包")

try:
    import sionna
    check(f"sionna {sionna.__version__}")
except Exception as e:
    fail("sionna import", str(e))
    sys.exit(1)

try:
    import sionna.rt
    from sionna.rt import __version__ as rt_ver
    check(f"sionna-rt {rt_ver}")
except Exception as e:
    fail("sionna.rt import", str(e))
    sys.exit(1)

from sionna.rt import (
    PathSolver,
    PlanarArray,
    Receiver,
    Transmitter,
    load_scene,
)

# ── 4. 场景加载测试 ─────────────────────────────────────────────────────
section("4. 场景加载")

# 4a. floor_wall
try:
    scene_fw = load_scene(sionna.rt.scene.floor_wall)
    check("floor_wall 场景加载成功")
except Exception as e:
    fail("floor_wall 场景加载", str(e))

# 4b. etoile (Paris)
try:
    scene_etoile = load_scene(sionna.rt.scene.etoile)
    check("etoile (Paris) 场景加载成功")
except Exception as e:
    fail("etoile 场景加载", str(e))

# ── 5. 天线阵列配置 ────────────────────────────────────────────────────
section("5. 天线阵列配置")

scene = scene_fw   # 使用 floor_wall 做后续测试
scene.frequency = 28e9

try:
    scene.tx_array = PlanarArray(
        num_rows=1, num_cols=1,
        vertical_spacing=0.5, horizontal_spacing=0.5,
        pattern="iso", polarization="V",
    )
    scene.rx_array = PlanarArray(
        num_rows=8, num_cols=1,
        vertical_spacing=0.5, horizontal_spacing=0.5,
        pattern="iso", polarization="V",
    )
    check("TX/RX PlanarArray 配置成功")
except Exception as e:
    fail("PlanarArray 配置", str(e))

# ── 6. 设备放置 ─────────────────────────────────────────────────────────
section("6. 发射/接收设备放置")

BS_POS = np.array([0.0, 0.0, 1.5], dtype=np.float32)
UE_POS = np.array([5.0, 0.0, 1.5], dtype=np.float32)

try:
    scene.add(Receiver(
        name="bs",
        position=BS_POS,
        orientation=[0, 0, 0],
    ))
    scene.add(Transmitter(
        name="ue",
        position=UE_POS,
        orientation=[0, 0, 0],
        power_dbm=0.0,
    ))
    check(f"BS @ {BS_POS}, UE @ {UE_POS}")
except Exception as e:
    fail("设备放置", str(e))

# ── 7. 射线追踪路径求解 ────────────────────────────────────────────────
section("7. 射线追踪路径求解")

solver = PathSolver()

try:
    paths = solver(
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
    check("路径求解完成 (max_depth=5)")
except Exception as e:
    fail("路径求解失败", str(e))
    sys.exit(1)

# ── 8. 路径 & 交互点统计 ────────────────────────────────────────────────
section("8. 路径 & 交互统计")

try:
    # SionnaRT 2.x 通过 vertices/sources/targets 暴露路径几何信息
    n_paths = len(paths.vertices)
    n_sources = len(paths.sources)
    n_targets = len(paths.targets)
    check(f"总路径数 (vertices): {n_paths}")
    check(f"  sources: {n_sources},  targets: {n_targets}")
except Exception as e:
    fail("路径统计", str(e))

# ── 9. CIR 生成 ──────────────────────────────────────────────────────────
section("9. CIR (信道冲激响应) 生成")

FS = 122.88e6  # 采样率

try:
    a, tau = paths.cir(sampling_frequency=FS, normalize_delays=False, out_type="numpy")
    check(f"CIR 生成成功 — shape: a={list(a.shape)}, tau={list(tau.shape)}")

    # 提取第一条天线对的 CIR
    a0 = np.asarray(a[0, :, 0, 0, :, 0], dtype=np.complex64)
    tau0 = np.asarray(tau[0, :, 0, 0, :], dtype=np.float32)
    path_power = np.sum(np.abs(a0) ** 2, axis=0)
    check(f"有效路径功率: {path_power.tolist()}")

    # 检查是否有有效功率
    total_power = float(np.sum(path_power))
    if total_power > 0:
        check(f"总接收功率 > 0 ({total_power:.2e})")
    else:
        fail("总接收功率为 0")
except Exception as e:
    fail("CIR 生成", str(e))
    sys.exit(1)

# ── 10. Doppler 频移提取（移动 UE） ─────────────────────────────────────
section("10. Doppler 频移验证（UE 匀速移动）")

FC = 28e9
C0 = 3e8
DT = 0.5e-3   # 0.5 ms 采样间隔
V_UE = 10.0   # 10 m/s 沿 x 轴
N_STEPS = 20

try:
    phases = []
    base_ue_pos = UE_POS.copy()

    for step in range(N_STEPS):
        ue_pos = np.array([base_ue_pos[0] + V_UE * step * DT, base_ue_pos[1], base_ue_pos[2]], dtype=np.float32)
        scene.get("ue").position = ue_pos

        paths_step = solver(
            scene,
            max_depth=5,
            los=True, specular_reflection=True, diffuse_reflection=True,
            diffraction=False, edge_diffraction=False, refraction=False,
            synthetic_array=False,
        )
        a_s, _ = paths_step.cir(sampling_frequency=FS, normalize_delays=False, out_type="numpy")
        a_s0 = np.asarray(a_s[0, :, 0, 0, :, 0], dtype=np.complex64)

        # 找最强路径
        pw = np.sum(np.abs(a_s0) ** 2, axis=0)
        best_idx = int(np.argmax(pw))
        phases.append(np.angle(a_s0[0, best_idx]))

    phases = np.array(phases, dtype=np.float64)

    # 相位解缠 & 线性拟合
    phases_unwrapped = np.unwrap(phases)

    # 用线性回归从相位斜率提取 Doppler
    t = np.arange(N_STEPS) * DT
    A = np.stack([t, np.ones_like(t)], axis=1)
    slope, _ = np.linalg.lstsq(A, phases_unwrapped, rcond=None)[0]
    f_doppler_measured = slope / (2 * np.pi)

    # 理论值: v_radial > 0 表示接近 (距离减小), v_radial < 0 表示远离
    # UE 沿 +x 移动, BS 在原点 → 距离增大 → Doppler 应为负
    dx = base_ue_pos[0] - BS_POS[0]
    d = np.linalg.norm(base_ue_pos[:2] - BS_POS[:2])
    v_radial_theory = V_UE * (-dx / d)   # 负号: UE 远离 BS
    f_doppler_theory = FC * v_radial_theory / C0

    check(f"Doppler 频移 — 测量: {f_doppler_measured:.1f} Hz")
    check(f"Doppler 频移 — 理论: {f_doppler_theory:.1f} Hz")

    relative_error = abs(f_doppler_measured - f_doppler_theory) / max(abs(f_doppler_theory), 1.0)
    if relative_error < 0.3:
        check(f"Doppler 验证通过 (相对误差 {relative_error*100:.1f}%)")
    else:
        fail(f"Doppler 验证偏差较大 (相对误差 {relative_error*100:.1f}%)")

except Exception as e:
    fail("Doppler 验证", str(e))

# ── 11. Mitsuba 渲染器检测 ─────────────────────────────────────────────
section("11. Mitsuba 渲染器（可选）")

try:
    import mitsuba
    check(f"mitsuba {mitsuba.__version__}")
except Exception as e:
    fail("mitsuba import（如需 RT 加速渲染请安装）", str(e))

# ── 12. PyTorch 检测 ────────────────────────────────────────────────────
section("12. PyTorch")

try:
    import torch
    check(f"pytorch {torch.__version__}")
    if torch.cuda.is_available():
        check(f"PyTorch CUDA: {torch.cuda.get_device_name(0)}")
    else:
        check("PyTorch CPU 模式")
except Exception as e:
    fail("pytorch import", str(e))

# ── 汇总 ─────────────────────────────────────────────────────────────────
section("验证结果汇总")

print(f"\n  总计: {PASS+FAIL} 项检查")
print(f"  通过: {PASS}")
print(f"  失败: {FAIL}")

if FAIL == 0:
    print(f"\n  ✓ SionnaRT 环境所有检查通过，可以正常使用。")
else:
    print(f"\n  ✗ 有 {FAIL} 项未通过，请根据上述 [FAIL] 条目排查。")

print(f"\n  脚本路径: {Path(__file__).resolve()}")
print()
