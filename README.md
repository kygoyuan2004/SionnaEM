# SionnaEM — 基于 Sionna RT 的 Micro-Doppler 仿真验证

## 项目目标

验证旋转运动产生的 **Micro-Doppler 效应能否集成入 Sionna RT 信道仿真平台**，为基于射线追踪的 UAV 检测/分类仿真提供可行的技术路线。

**核心策略**：RT 引擎只调用一次（处理静态场景），时变 Micro-Doppler 调制在 CIR 后处理阶段通过叠加正弦相位调制实现，Sionna RT 修改量 = 0 行。

## 目录结构

```
SionnaEM/
├── README.md                          # 本文件
├── requirements.txt                   # Python 依赖
│
├── src/                               # 核心源代码（~3,300 行）
│   ├── micro_doppler_modulator.py     # MicroDopplerModulator 核心模块
│   ├── verify_modulator.py            # 26 项系统验证
│   ├── micro_doppler_integration_demo.py  # Sionna RT 集成演示
│   ├── verify_sionna_rt_env.py        # 环境验证工具
│   ├── step1_baseline_doppler.py      # Step 1：标准 Doppler 基线
│   ├── step2_single_scatterer.py      # Step 2：单散射点 Micro-Doppler
│   └── step3_quadrotor_spectrogram.py # Step 3：四旋翼 UAV 谱图
│
├── docs/                              # 文档与报告
│   ├── 调研问题与思考框架.md
│   ├── Micro_Doppler_Validation_Plan.md
│   ├── 后续三步计划_第四至六步.md
│   ├── Micro_Doppler_Sionna_RT_三步验证报告.md / .pdf
│   ├── Step4_完成报告.md / .pdf
│   ├── Step5_完成报告.md / .pdf
│   ├── SionnaEM_项目成果汇报.pdf       # 综合成果汇报（含嵌入图示）
│   ├── SionnaEM_项目问答汇总.pdf       # 常见问答汇总
│   └── group_meeting_report.html / .pdf
│
├── papers/                            # 参考论文
│   ├── Micro-Doppler_Signature-Based_Detection...Small_UAV.pdf
│   ├── Micro-Doppler_Signature_Simulation_of_Multirotor_UAVs...Ray_Tracing.pdf
│   └── Micro_Doppler_Sionna_RT_调研报告.pdf
│
├── figures/                           # 产出图表（16 张）
│   ├── baseline_doppler_etoile.png
│   ├── micro_doppler_single_scatterer.png
│   ├── micro_doppler_quadrotor_spectrogram.png
│   ├── micro_doppler_uav_vs_noise.png
│   ├── micro_doppler_modulation_frequency.png
│   ├── micro_doppler_modulator_validation.png
│   ├── integration_summary.png
│   └── ... (共 16 张)
│
├── tools/                             # 辅助工具
│   ├── CIR_to_CFR/                    # CIR → CFR 转换工具
│   └── scene_create/                  # 场景数据集生成
│
├── sionna-large-radio-maps/           # Sionna 大尺度无线电地图库
└── RT_tutorial/                       # Sionna RT 教程与单元测试
```

## 六步推进计划

| 步骤 | 名称 | 核心产出 | 状态 |
|------|------|---------|------|
| Step 1 | 标准 Doppler 基线 | `step1_baseline_doppler.py` | ✓ 完成 |
| Step 2 | 单散射点 Micro-Doppler | `step2_single_scatterer.py` | ✓ 完成 |
| Step 3 | 四旋翼 UAV 谱图 | `step3_quadrotor_spectrogram.py` | ✓ 完成 |
| Step 4 | Modulator 模块化 | `micro_doppler_modulator.py` | ✓ 完成 |
| Step 5 | CIR 管道集成验证 | `micro_doppler_integration_demo.py` | ✓ 完成 |
| Step 6 | 论文对标 | 多 UAV 配置参数扫描 | 待开展 |

## 环境要求

| 组件 | 版本/型号 |
|------|----------|
| Python | 3.11.15 |
| Sionna RT | 2.0.1 |
| TensorFlow | 2.20.0 (GPU) |
| NumPy | 2.4.4 |
| SciPy | 1.17.1 |
| GPU | RTX 4090 (24 GB) |
| CUDA | 12.8 |

```bash
# 激活环境
conda activate sionna_rt

# 安装依赖
pip install -r requirements.txt
```

## 快速开始

### 运行 Modulator 验证（26 项测试）

```bash
cd src
python verify_modulator.py
```

### 运行 Sionna RT 集成演示（Step 5）

```bash
cd src
python micro_doppler_integration_demo.py                    # 全部 3 个场景
python micro_doppler_integration_demo.py --scenes paris_los # 仅自由空间
python micro_doppler_integration_demo.py --no-plots         # 跳过绘图
```

### 运行各步骤的独立脚本

```bash
cd src
python step1_baseline_doppler.py          # Step 1：标准 Doppler
python step2_single_scatterer.py          # Step 2：单散射点
python step3_quadrotor_spectrogram.py     # Step 3：四旋翼谱图
```

### 在代码中使用 MicroDopplerModulator

```python
import sys; sys.path.insert(0, 'src')
from micro_doppler_modulator import UAVMicroDopplerConfig, MicroDopplerModulator
import numpy as np

# 创建配置
config = UAVMicroDopplerConfig(
    carrier_freq=28e9,
    blade_radius=0.15,
    rotation_freq=100.0,
    body_velocity=(5.0, 0.0, 0.0),
)

# 创建调制器并生成信号
mod = MicroDopplerModulator(config)
t = np.arange(int(config.sampling_rate * 0.2)) / config.sampling_rate
signal = mod.generate_received_signal(t)

# 谱图分析
f, t_s, S_dB = mod.stft_spectrogram(signal)
harmonics = mod.detect_harmonics(signal)
print(f"Detected {len(harmonics)} harmonics")
```

## 关键验证结果

| 参数 | 理论值 | 实测值 | 误差 |
|------|--------|--------|------|
| 调制指数 β = 4πR/λ | 176.1 rad | 176.05 rad | 0.00% |
| 峰值频偏 ±β·f_rot | ±17.6 kHz | ±17.6 kHz | — |
| 标准 Doppler f_d | −933.98 Hz | −933.98 Hz | 0.00% |
| CIR-几何相位相关系数 | 1.0 | 1.000000 | — |
| 谐波间距 | 100 Hz | 100 Hz (14 次谐波) | — |

## 性能基准

| 指标 | 目标 | 实测 |
|------|------|------|
| RT 求解 | < 1 s | ~15 ms |
| CIR 调制 (10000 样本) | < 0.1 s | ~5.5 ms |
| 总管线 (RT + 调制 + STFT) | < 2 s | ~65 ms |
| 内存占用 | < 500 MB | < 200 MB |

## 已验证的核心思想

1. **正弦相位调制模型** — 旋转叶片的往返距离变化是正弦函数，相位调制指数 β = 4πR/λ（单基地往返）
2. **非侵入式集成** — RT 引擎只调用一次，时变调制在 CIR 后处理阶段叠加，Sionna 代码修改量为 0
3. **多散射点叠加模型** — 四旋翼 UAV = 1 机身 + 8 叶片尖端散射点的加权叠加
4. **远场共享多径结构** — 散射点间距远小于雷达距离时，所有散射点共享同一组 RT 路径

## 局限性与改进方向

- 近场近似误差（< 50 m 时明显）
- RCS 方向图缺失（恒定幅度权重）
- 无动态遮挡效应
- 点散射体近似（非分布散射）
- 静态多径结构（时延不随时间变化）
- 无叶片弹性变形

详见 `docs/SionnaEM_项目问答汇总.pdf` 第三章。
