# SionnaEM — 基于 Sionna RT 的 Micro-Doppler 仿真与 UAV 信道建模

## 项目概述

本项目旨在验证**旋转运动产生的 Micro-Doppler 效应能否集成入 Sionna RT 信道仿真平台**，为基于射线追踪的 UAV 检测/分类/信道建模仿真提供可行的技术路线。

**核心策略**：RT 引擎只调用一次（处理静态场景），时变 Micro-Doppler 调制在 CIR 后处理阶段通过叠加正弦相位调制实现，Sionna RT 代码修改量为 0 行。

**延伸工作**：在 Micro-Doppler 验证基础上，进一步构建了 Blender → Sionna RT → Pathloss / CSI 的端到端仿真管线，支持自定义 3D 场景导入、多基站无线电图生成和信道状态信息提取。

---

## 目录结构

```
SionnaEM/
├── README.md                                    # 本文件
├── requirements.txt                             # Python 依赖清单
├── .gitignore                                   # Git 忽略规则
│
├── src/                                         # 核心源代码（14 个脚本）
│   │
│   ├── step1_baseline_doppler.py                # [Step 1] 标准 Doppler 基线验证
│   ├── step2_single_scatterer.py                # [Step 2] 单散射点 Micro-Doppler 验证
│   ├── step3_quadrotor_spectrogram.py           # [Step 3] 四旋翼 UAV 谱图生成
│   ├── micro_doppler_modulator.py               # [Step 4] MicroDopplerModulator 核心模块
│   ├── verify_modulator.py                      # [Step 4] Modulator 系统验证（26 项测试）
│   ├── micro_doppler_integration_demo.py        # [Step 5] CIR 管道集成演示
│   ├── step6_static_drone_scene.py              # [Step 6] 静态无人机场景信道图
│   ├── step7_dynamic_drone_demo.py              # [Step 7] 动态无人机演示
│   ├── step8_blender_scene_interface.py         # [Step 8] Blender 场景接口
│   ├── step9_pipeline_pathloss_csi.py           # [Step 9] 完整管线：Pathloss + CSI
│   ├── verify_sionna_rt_env.py                  # 环境验证工具
│   ├── paper_fig2_style_mds.py                  # 论文 Fig.2 风格 Micro-Doppler 谱图复现
│   ├── rt_micro_doppler_vs_standard_doppler.py  # RT 原生 Micro-Doppler vs 标准 Doppler 对比
│   └── rt_micro_doppler_vs_standard_doppler_single.py  # 单旋翼版本
│
├── pipeline/                                    # 管线输出目录
│   ├── data/                                    #   管线输出数据 (.npz)
│   └── figures/                                 #   管线输出图表
│
├── tools/                                       # 辅助工具集
│   │
│   ├── blender_to_sionna/                       # Blender → Sionna RT 场景导入
│   │   ├── generate_scene_xml.py                #   场景 XML 生成器
│   │   └── README.md                            #   工具使用说明
│   │
│   ├── cir_to_cfr/                              # CIR → CFR 转换工具
│   │   ├── dataset_CIR_to_CFR.py                #   CIR → OFDM CFR 张量重建
│   │   └── Hgt_transform_to_HLS.py              #   CIR → CFR → 导频 LS 估计
│   │
│   ├── scene_dataset/                           # 场景数据集批量生成
│   │   ├── scene_dataset_create_normal.py       #   通用 RT 场景
│   │   └── scene_dataset_create_paris.py        #   Paris 场景专用
│   │
│   ├── md_to_pdf.py                             # Markdown → PDF 转换器
│   ├── modify_ppt.py                            # PPT 自动修改工具
│   └── parameter_report_pdf.py                  # 参数报告 PDF 生成器
│
├── docs/                                        # 项目文档与阶段报告
│   │
│   ├── research_questions.md                    # 初期调研问题梳理
│   ├── Micro_Doppler_Validation_Plan.md          # 三步验证计划
│   ├── three_step_validation_report.md/.pdf      # Step 1-3 验证结果
│   ├── steps_4to6_plan.md                       # Step 4-6 工作规划
│   ├── Step4_完成报告.md/.pdf                    # Step 4 完成报告
│   ├── Step5_完成报告.md/.pdf                    # Step 5 完成报告
│   ├── SionnaEM_Project_Structure_Analysis.md/.pdf # 项目结构分析
│   ├── project_achievement_report.pdf            # 综合成果汇报
│   ├── project_qa_summary.pdf                    # 技术问答汇总
│   ├── Technical_QA.md/.pdf                     # 技术问题问答
│   ├── Work_Summary.md/.pdf                     # 工作总结
│   ├── Blender_Sionna_MicroDoppler_Chain.md/.pdf  # Blender 全链路技术文档
│   ├── Blender_Tutorial_Beginner.md/.pdf         # Blender 入门教程
│   ├── BS_Parameter_Reference.md/.pdf            # 基站参数参考
│   ├── Collaboration_Platform_Research.md/.pdf   # 协作平台调研
│   ├── group_meeting_report.html/.pdf            # 组会汇报
│   ├── parameter_report.md/.pdf                  # 参数报告
│   ├── parameter_report_tables.md/.pdf           # 参数报告（表格版）
│   ├── pipeline_report.md                        # 管线工作总结
│   └── variable_reference.md                     # 变量/参数快速参考
│
├── figures/                                     # 产出图表（~18 张 PNG）
│   ├── baseline_doppler_etoile.png              # Step 1：标准 Doppler 基线
│   ├── baseline_doppler_wrapped_phase.png        # Step 1：包裹相位
│   ├── micro_doppler_single_scatterer.png        # Step 2：单散射点谱图
│   ├── micro_doppler_quadrotor_spectrogram.png   # Step 3：四旋翼谱图
│   ├── micro_doppler_modulator_validation.png    # Step 4：Modulator 交叉验证
│   ├── modulator_verification.png                # Step 4：多场景验证
│   ├── integration_paris_los.png                 # Step 5：Paris LOS 集成
│   ├── integration_paris_multipath.png           # Step 5：Paris 多径集成
│   ├── integration_floor_wall.png                # Step 5：floor_wall 场景
│   ├── integration_summary.png                   # Step 5：集成管线总结
│   ├── micro_doppler_cir_diagnostics.png         # CIR 调制诊断
│   ├── micro_doppler_modulation_analysis.png     # 调制深度和谐波分析
│   ├── micro_doppler_modulation_frequency.png    # 调制频率成分
│   ├── micro_doppler_uav_vs_noise.png            # UAV 检测对比度
│   ├── micro_doppler_stationary_vs_moving.png    # 静止 vs 运动 UAV
│   ├── micro_doppler_vs_standard_doppler.png     # Micro-Doppler vs 标准 Doppler
│   ├── single_rotor_micro_doppler_vs_standard_doppler.png  # 单旋翼 RT 对比
│   └── paper_fig2_style_microhelicopter_vs_quadcopter.png  # 论文风格对比
│
├── ppt_assets_uav/                              # 组会 PPT 素材文档
│   ├── asset_manifest.md                        #   素材清单
│   ├── image_explanations.html                  #   图片图解说明
│   └── UAV_MicroDoppler_图片图解说明.pdf          #   图解说明 PDF
│
├── papers/                                      # 参考论文
│   ├── Micro-Doppler_Signature-Based_Detection_Classification_and_Localization_of_Small_UAV.pdf
│   ├── Micro-Doppler_Signature_Simulation_of_Multirotor_UAVs_Using_Ray_Tracing.pdf
│   └── Micro_Doppler_Sionna_RT_调研报告.pdf
│
├── RT_tutorial/                                 # Sionna RT 官方教程（.gitignore 排除）
└── sionna-large-radio-maps/                     # Sionna 大尺度无线电地图库（.gitignore 排除）
```

---

## 工作推进计划

| 步骤 | 名称 | 核心产出 | 关键验证 | 状态 |
|------|------|---------|---------|------|
| Step 1 | 标准 Doppler 基线 | `step1_baseline_doppler.py` | UE 匀速运动 Doppler 频移 vs 理论值 | ✓ 完成 |
| Step 2 | 单散射点 Micro-Doppler | `step2_single_scatterer.py` | β = 4πR/λ 正弦相位调制模型 | ✓ 完成 |
| Step 3 | 四旋翼 UAV 谱图 | `step3_quadrotor_spectrogram.py` | 8 叶片 + 1 机身叠加的直升机特征谱图 | ✓ 完成 |
| Step 4 | Modulator 模块化 | `micro_doppler_modulator.py` | 26 项系统验证全部通过 | ✓ 完成 |
| Step 5 | CIR 管道集成验证 | `micro_doppler_integration_demo.py` | 3 场景 RT + 调制 + STFT 完整管线 | ✓ 完成 |
| Step 6 | 静态无人机场景 | `step6_static_drone_scene.py` | 多基站双频段信道图与路径损耗图 | ✓ 完成 |
| Step 7 | 动态无人机演示 | `step7_dynamic_drone_demo.py` | Mode A（逐步RT）和 Mode B（快速调制） | ✓ 完成 |
| Step 8 | Blender 场景接口 | `step8_blender_scene_interface.py` | 三种场景来源统一加载 | ✓ 完成 |
| Step 9 | 端到端管线 | `step9_pipeline_pathloss_csi.py` | Blender → RT → Pathloss + CSI 完整管线 | ✓ 完成 |
| Step 10 | 论文对标与参数扫描 | 多 UAV 配置参数扫描 | — | 待开展 |

---

## 环境要求

| 组件 | 版本/型号 | 说明 |
|------|----------|------|
| Python | 3.11.15 | Conda 环境 `sionna_rt` |
| Sionna | 2.0.1 | 信道仿真框架 |
| Sionna RT | 2.0.1 | 射线追踪模块 |
| TensorFlow | 2.20.0 (GPU) | 深度学习后端 |
| Keras | 3.14.1 | 高级 API |
| Mitsuba | 3.8.0 | 光线追踪渲染引擎 |
| Dr.JIT | 1.3.1 | Mitsuba 依赖 |
| NumPy | ≥ 2.0 | 科学计算 |
| SciPy | ≥ 1.15 | 信号处理（STFT/FFT） |
| Matplotlib | ≥ 3.9 | 可视化 |
| WeasyPrint | ≥ 68 | PDF 报告生成（可选） |
| GPU | NVIDIA RTX 4090 (24 GB) | 射线追踪加速 |
| CUDA | 12.8 | GPU 计算 |

```bash
# 激活环境
conda activate sionna_rt

# 安装依赖
pip install -r requirements.txt
```

---

## 快速开始

### 0. 环境验证

确认 Sionna RT 各组件的安装和可用性：

```bash
cd src
python verify_sionna_rt_env.py
```

### 1. 运行 Modulator 验证（26 项测试）

验证 MicroDopplerModulator 的正确性（A/B/C 三层验证）：

```bash
cd src
python verify_modulator.py
```

### 2. 运行 Sionna RT 集成演示（Step 5）

将 Modulator 嵌入 Sionna RT CIR 管线：

```bash
cd src
python micro_doppler_integration_demo.py                    # 全部 3 个场景
python micro_doppler_integration_demo.py --scenes paris_los # 仅自由空间
python micro_doppler_integration_demo.py --no-plots         # 跳过绘图
```

### 3. 运行各步骤的独立脚本

```bash
cd src
python step1_baseline_doppler.py               # Step 1：标准 Doppler
python step2_single_scatterer.py               # Step 2：单散射点 Micro-Doppler
python step3_quadrotor_spectrogram.py          # Step 3：四旋翼 UAV 谱图
python step6_static_drone_scene.py             # Step 6：静态无人机场景信道图
python step6_static_drone_scene.py --freq 3.5  #   仅 3.5 GHz
python step6_static_drone_scene.py --freq 28   #   仅 28 GHz
python step7_dynamic_drone_demo.py             # Step 7：动态无人机演示（Mode A+B）
python step7_dynamic_drone_demo.py --mode fast #   仅 Mode B（快速调制）
```

### 4. RT 原生 Micro-Doppler vs 标准 Doppler 对比

不使用 Modulator，直接用多次 RT 求解实现"最诚实的"物理对比：

```bash
cd src
python rt_micro_doppler_vs_standard_doppler.py        # 四旋翼（8 叶片）
python rt_micro_doppler_vs_standard_doppler_single.py  # 单旋翼（2 叶片直升机）
```

### 5. 端到端管线（Step 8-9）

```bash
cd src

# Step 8: 加载场景（三种来源）
python step8_blender_scene_interface.py

# Step 9: 完整管线 — 输出到 pipeline/data/ 和 pipeline/figures/
python step9_pipeline_pathloss_csi.py                              # 默认：双频段
python step9_pipeline_pathloss_csi.py --freq 28                    # 仅 28 GHz
python step9_pipeline_pathloss_csi.py --ofdm                       # 启用 OFDM CFR
python step9_pipeline_pathloss_csi.py --scene-xml my_scene.xml     # 自定义场景
python step9_pipeline_pathloss_csi.py --ply-dir ./meshes --material-map ./materials.json
```

### 6. 在代码中使用 MicroDopplerModulator

```python
import sys; sys.path.insert(0, 'src')
from micro_doppler_modulator import UAVMicroDopplerConfig, MicroDopplerModulator
import numpy as np

# 创建 UAV 配置
config = UAVMicroDopplerConfig(
    carrier_freq=28e9,       # 28 GHz 载波
    blade_radius=0.15,       # 叶片半径 0.15 m
    rotation_freq=100.0,     # 转速 100 Hz ≈ 6000 RPM
    body_velocity=(5.0, 0.0, 0.0),  # 机身速度 5 m/s
)

# 创建调制器
mod = MicroDopplerModulator(config)

# 生成接收信号
t = np.arange(int(config.sampling_rate * 0.2)) / config.sampling_rate
signal = mod.generate_received_signal(t)

# STFT 谱图分析
f, t_s, S_dB = mod.stft_spectrogram(signal)

# 检测谐波
harmonics = mod.detect_harmonics(signal)
print(f"检测到 {len(harmonics)} 次谐波")

# 与 Sionna RT CIR 集成
a_dyn, tau_dyn = mod.modulate_cir(a_static, tau_static, t)
```

---

## 关键验证结果

| 参数 | 理论值 | 实测值 | 误差 |
|------|--------|--------|------|
| 调制指数 β = 4πR/λ | 176.1 rad | 176.05 rad | 0.00% |
| 峰值频偏 ±β·f_rot | ±17.6 kHz | ±17.6 kHz | — |
| 标准 Doppler f_d | −933.98 Hz | −933.98 Hz | 0.00% |
| CIR-几何相位相关系数 | 1.0 | 1.000000 | — |
| 谐波间距 | 100 Hz | 100 Hz（14 次谐波） | — |

---

## 性能基准

| 指标 | 目标 | 实测 |
|------|------|------|
| RT 求解（无场景加载） | < 1 s | ~15 ms |
| CIR 调制（10000 样本） | < 0.1 s | ~5.5 ms |
| 总管线（RT + 调制 + STFT） | < 2 s | ~65 ms |
| 内存占用 | < 500 MB | < 200 MB |

---

## 已验证的核心思想

1. **正弦相位调制模型** — 旋转叶片的往返距离变化是正弦函数，相位调制指数 β = 4πR/λ（单基地往返）。该模型可精确描述旋转散射体在 CIR 相位中引入的调制
2. **非侵入式集成** — RT 引擎只调用一次，时变调制在 CIR 后处理阶段叠加，Sionna 代码修改量为 0 行。这确保了与 Sionna 上游更新的完全兼容
3. **多散射点叠加模型** — 四旋翼 UAV = 1 机身 + 8 叶片尖端散射点的加权叠加。每个散射点的相位调制独立计算后叠加，产生完整的 Micro-Doppler 谱图
4. **远场共享多径结构** — 散射点间距远小于雷达距离时，所有散射点共享同一组 RT 路径（时延/角度信息相同，仅相位不同），证实了"一次 RT + 后处理调制"方案的理论正确性
5. **双频段可扩展性** — 管线支持 3.5 GHz（Sub-6）和 28 GHz（mmWave）两个频段，可复用同一场景只需修改 `scene.frequency`
6. **Blender → Sionna 场景管线** — 支持从 Blender 3D 模型直接导出 PLY → 生成 Mitsuba XML → Sionna RT 加载的完整工作流，使仿真可脱离 Sionna 内置场景限制

---

## 局限性与改进方向

### 物理模型局限
- **近场近似误差**（< 50 m 时明显）— 当前模型假设所有散射点共享同一组路径时延，在近场条件下该假设引入相位误差
- **RCS 方向图缺失** — 当前使用恒定幅度权重，未考虑叶片散射截面随入射角的变化
- **无动态遮挡效应** — 叶片旋转过程中可能被机身或其他叶片遮挡，当前模型未建模
- **点散射体近似** — 将整个叶片近似为尖端单点散射，忽略了分布式散射贡献
- **无叶片弹性变形** — 高速旋转时叶片的弹性形变未被建模

### 多径模型局限
- **静态多径结构** — 时延不随时间变化，但实际中 UAV 的移动会导致路径时延的缓变
- **无环境动态交互** — 未考虑 UAV 旋转叶片对周围环境反射模式的改变

详见 `docs/project_qa_summary.pdf` 第三章和 `docs/Technical_QA.md`。

---

## 论文参考

- Sun et al., "Micro-Doppler Signature-Based Detection, Classification, and Localization of Small UAV With Long Short-Term Memory Neural Network," *IEEE TGRS*, 2021.
- 相关 Micro-Doppler / 射线追踪仿真论文（见 `docs/` 中引用）

---

## 项目贡献者

- 主要开发者：kygoyuan2004
- 工作单位：SionnaEM 课题组
- 项目周期：2026 年 4 月 — 至今
