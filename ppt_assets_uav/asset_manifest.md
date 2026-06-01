# PPT 素材清单 — 第二部分：UAV 微多普勒感知与 Sionna RT 仿真

> 组会 PPT 第 16–22 页素材索引与使用说明
> 生成日期：2026-05-20

---

## 素材总览

| 文件 | 来源 | 大小 | 分辨率 | DPI |
|------|------|------|--------|-----|
| `p16_quadrotor_microdoppler_spectrogram.png` | figures/micro_doppler_quadrotor_spectrogram.png | 372K | 2502×1680 | 200 |
| `p16_uav_detection_contrast.png` | figures/micro_doppler_uav_vs_noise.png | 514K | 2545×987 | 200 |
| `p20_baseline_doppler_validation.png` | figures/baseline_doppler_etoile.png | 275K | 2385×1575 | 200 |
| `p20_integration_pipeline_summary.png` | figures/integration_summary.png | 919K | 2704×2214 | 200 |
| `p20_modulator_cross_validation.png` | figures/micro_doppler_modulator_validation.png | 289K | 2390×892 | 200 |
| `p21_single_scatterer_sinusoidal.png` | figures/micro_doppler_single_scatterer.png | 617K | 2156×1680 | 200 |
| `p21_quadrotor_spectrogram_full.png` | figures/micro_doppler_quadrotor_spectrogram.png | 372K | 2502×1680 | 200 |
| `p21_modulation_frequency_harmonics.png` | figures/micro_doppler_modulation_frequency.png | 342K | 2385×1379 | 200 |
| `p21_stationary_vs_moving_uav.png` | figures/micro_doppler_stationary_vs_moving.png | 258K | 2545×987 | 200 |
| `p22_standard_vs_microdoppler_comparison.png` | figures/micro_doppler_vs_standard_doppler.png | 237K | 2585×1379 | 200 |
| `p22_project_integration_summary.png` | figures/integration_summary.png | 919K | 2704×2214 | 200 |

---

## 逐页使用说明

### 第 16 页：为什么 UAV 微多普勒感知值得放进这次组会

**推荐主图**：`p16_quadrotor_microdoppler_spectrogram.png`

- **图中内容**：四旋翼 UAV 的完整 4 面板图 — (a) UAV 几何俯视图 / (b) 全带 Micro-Doppler 时频谱图（inferno 色图，±20 kHz）/ (c) 低频边带放大图（±300 Hz，展示 100 Hz 边带间距）/ (d) Doppler 剖面与谐波标注
- **怎么讲**："这是我们在 Sionna RT 框架下生成的第一个四旋翼 UAV 完整 Micro-Doppler 谱图。fc=28 GHz，R=0.15 m，f_rot=100 Hz（6000 RPM），8 个叶片尖端+1 个机身共 9 个散射点。谱图中清晰可见 ±17.6 kHz 宽带扩展、467 Hz 机身 Doppler 线、100 Hz 边带谐波间距——这就是文献中经典的 helicopter signature。"
- **使用建议**：直接使用。如果 PPT 空间有限，可只取子图 (b) 全带谱图作为主视觉，这是最有视觉冲击力的部分。
- **备选图**：`p16_uav_detection_contrast.png`（UAV 有/无对比，用于强调"UAV 信号与纯噪声极易区分"）

**推荐 caption**：「四旋翼 UAV 的 Micro-Doppler 时频谱图 — 经典的 helicopter signature，±17.6 kHz 宽带扩展 + 100 Hz 边带谐波」

---

### 第 17 页：论文一解决了什么问题

**本项目不包含论文一的原始图表**。论文一 "Micro-Doppler Signature-Based Detection, Classification, and Localization of Small UAV With LSTM"（IEEE TGRS 2021, 16 页）为第三方论文，其系统图、方法流程图需手动从 PDF 中截图提取。

**建议从论文一 PDF 手动提取的图**：
- 论文一 Fig.1：双基地雷达系统几何图（TX/RX/UAV 空间关系）
- 论文一 Fig.2 或方法总览图：检测/分类/定位三任务框图
- 论文一系统模型图：UAV 微多普勒信号模型示意

**提取方法**：用 PDF 阅读器打开 `papers/Micro-Doppler_Signature-Based_Detection...Small_UAV.pdf`，截取目标图表区域，保存为 PNG。

---

### 第 18 页：论文一的核心链路是什么

**本项目不包含论文一的原始图表**。建议从论文一 PDF 手动提取：

- 论文一的 MDSUS 方法流程图（spectral subtraction → EMD → STFT → PCA → LSTM）
- 论文一的 LSTM 网络结构图
- 论文一的微多普勒信号处理链路示意图

**提取后建议**：如果原图文字过小或不清晰，建议用 PPT 重画为简洁流程图，保留核心模块名称和箭头关系。

---

### 第 19 页：论文一的最好证据是什么

**本项目不包含论文一的原始图表**。建议从论文一 PDF 手动提取：

- 检测 F1-score 对比表/图
- 分类准确率混淆矩阵或对比表
- AoA 定位误差结果
- 实时性/计算复杂度表格

**本项目可用的辅助图**（放备份页）：
- `p16_uav_detection_contrast.png` — 用于侧面佐证"UAV 信号的可检测性很好"
- `p21_modulation_frequency_harmonics.png` — 用于说明"包络调制频谱的谐波特征可以支持分类"

---

### 第 20 页：论文二的动机是数据稀缺与泛化困难

**推荐主图**：`p20_integration_pipeline_summary.png`

- **图中内容**：Step 5 三场景集成验证汇总 — (a-c) Paris LOS / Paris 多径 / floor_wall 三场景谱图 / (d) 三场景 + 解析参考 Doppler 剖面对比 / (e) 验证指标柱状图（相关性、频谱扩展）/ (f) 性能基准表格
- **怎么讲**："论文二的动机是实测 micro-Doppler 数据获取困难且场景泛化性差。我们的方案：Sionna RT 做一次静态射线追踪得到 CIR（仅 15 ms），然后用 MicroDopplerModulator 在 CIR 后处理阶段叠加正弦相位调制（5.5 ms），生成带多径环境的时变 micro-Doppler 信号。整个管线耗时 < 70 ms，Sionna RT 引擎修改量为 0 行。"
- **使用建议**：建议裁剪使用。子图 (a-c) 可作为论文二的 Sionna RT 仿真流程图佐证，子图 (d) 对比图最有说服力。如果 PPT 空间有限，只取 (a-c) 三场景谱图行。
- **备选图**：
  - `p20_baseline_doppler_validation.png` — 标准 Doppler 基线（Step 1），用于说明 RT 链路从基础 Doppler 验证起步
  - `p20_modulator_cross_validation.png` — Modulator 交叉验证（调制谱图 + 谐波检测），放备份页

**推荐 caption**：「基于 Sionna RT 的 Micro-Doppler 仿真管线 — 一次 RT 求解 + CIR 后处理调制 = 多场景可泛化的 UAV 微多普勒数据生成」

---

### 第 21 页：论文二最有价值的是物理可解释结果

本页是本部分最"图多"的一页，建议选 2-3 张最核心的放在正文，其余放备份页。

**推荐主图 1**：`p21_quadrotor_spectrogram_full.png`

- 同 p16 主图。如果第 16 页已使用，这里可用子图 (c) 低频放大 + 子图 (d) Doppler 剖面，聚焦于物理可解释性：
  - 100 Hz 边带间距 → 对应 f_rot
  - 800 Hz 叶片闪烁 → 对应 N_blades × f_rot = 8×100
  - ±17.6 kHz 频偏 → 对应 β×f_rot = 176.1×100
- **建议 caption**：「谱图中每一个结构都有明确的物理对应：边带间距 = 旋翼转速 f_rot，频偏范围 = 调制指数 β × f_rot，闪烁频率 = 叶片数 × f_rot」

**推荐主图 2**：`p21_single_scatterer_sinusoidal.png`

- **图中内容**：单旋转散射点四面板 — (a) 3D 旋转轨迹 / (b) 正弦相位调制 φ(t) / (c) 瞬时频率正弦振荡 / (d) CIR 相位与几何预测验证
- **怎么讲**："我们从最简单的单散射点做起，验证最基本的物理：一个以 f_rot=100 Hz 旋转的散射点，其相位调制是严格的正弦函数 φ_mod(t)=β·sin(2π·f_rot·t)，调制指数 β=4πR/λ≈176 rad，与理论值误差 0.00%。Sionna RT 的 CIR 相位与几何预测值相关系数 = 1.000000。"
- **使用建议**：建议裁剪。子图 (b) 相位调制曲线 + 子图 (c) 瞬时频率曲线 最具物理可解释性。子图 (a) 3D 轨迹和 (d) CIR 验证可作为备份。
- **建议 caption**：「单旋转散射点的 Micro-Doppler 物理验证 — 相位调制严格遵循 φ_mod(t) = β·sin(2πf_rot·t)，β 实测 176.05 rad vs 理论 176.05 rad」

**推荐主图 3**：`p21_modulation_frequency_harmonics.png`

- **图中内容**：(a) 接收信号包络 |s(t)| 展示 12.5 ms 叶片闪烁周期 / (b) 包络调制频谱展示 f_rot 的各次谐波，800 Hz 处主峰（8 叶片×100 Hz）
- **建议 caption**：「包络调制频谱揭示叶片闪烁频率 = 800 Hz（8 叶片×100 Hz），偶数谐波占优源于四旋翼的对称几何结构」

**备选图 / 备份页**：
- `p21_stationary_vs_moving_uav.png` — 静止 vs 移动 UAV 谱图对比（展示机身 Doppler 平移但不改变微多普勒调制结构）

---

### 第 22 页：两篇 UAV 论文如何互补

**推荐主图**：`p22_project_integration_summary.png`

- 同 p20 的 integration_summary，这里侧重"项目总结"视角
- **怎么讲**："论文一解决的是'有了实测数据后怎么做检测分类定位'，论文二解决的是'如何用 Sionna RT 生成可泛化的仿真数据'。两者互补：论文一的 MDSUS 方法需要大量训练数据 → 论文二的 RT 仿真管线可以按需生成不同场景、不同 UAV 配置的训练数据。我们的工作打通了从 RT 仿真到 Micro-Doppler 特征生成的完整链路。"
- **建议 caption**：「实测任务闭环（论文一）+ 仿真数据生成（论文二）= 完整的 UAV 微多普勒感知研究链路」

**推荐辅图**：`p22_standard_vs_microdoppler_comparison.png`

- **图中内容**：标准 Doppler（线性相位、常数频率）vs Micro-Doppler（正弦调制相位、振荡频率）四面板并排对比
- **建议 caption**：「标准 Doppler：匀速运动 → 线性相位 → 常数频移；Micro-Doppler：旋转部件 → 正弦调制相位 → 周期性频移振荡」

---

## 其它可用但未复制的素材

以下素材未复制到 ppt_assets_uav/，但可视需要手动取用：

| 文件 | 内容 | 可能用途 |
|------|------|---------|
| `figures/micro_doppler_modulation_analysis.png` | 三相分解图（完整相位、模型残差、频率误差） | 备份页，展示 β 拟合精度 |
| `figures/micro_doppler_cir_diagnostics.png` | CIR 功率稳定性、路径数诊断 | 备用，展示仿真鲁棒性 |
| `figures/baseline_doppler_wrapped_phase.png` | 原始缠绕相位诊断 | 一般不需要 |
| `figures/integration_paris_los.png` | Paris LOS 单场景 2×3 详细对比 | 备份页，展示单场景细节 |
| `figures/integration_paris_multipath.png` | Paris 多径单场景 2×3 详细对比 | 备份页，展示多径效应 |
| `figures/integration_floor_wall.png` | floor_wall 单场景 2×3 详细对比 | 备份页 |
| `figures/modulator_verification.png` | Modulator 三面板验证（谱图+谐波+β∝R） | 备份页，展示模块验证 |
| `docs/group_meeting_report.pdf` | 组会报告 PDF | 参考整体汇报逻辑 |
| `docs/SionnaEM_项目成果汇报.pdf` | 项目成果汇报（含嵌入图示） | 参考既有汇报材料 |
| `docs/Micro_Doppler_Sionna_RT_三步验证报告.pdf` | 三步验证完整报告 | 提取验证结果表格 |

---

## 论文图表提取指南

两篇核心论文位于 `papers/`，均需手动提取图表：

### 论文一
`Micro-Doppler_Signature-Based_Detection_Classification_and_Localization_of_Small_UAV.pdf`
- IEEE TGRS 2021, 16 页, 双栏排版
- 需要提取的图：系统模型图、MDSUS 流程图、LSTM 结构图、结果对比表/图

### 论文二
`Micro-Doppler_Signature_Simulation_of_Multirotor_UAVs_Using_Ray_Tracing.pdf`
- IEEE ICCT 2025, 6 页, 双栏排版
- 需要提取的图：Sionna RT 仿真流程图、系统架构图、Fig.4/5/6 物理规律图
- 本项目生成的仿真图可直接替代/补充论文二的实验图

### 提取方法
```bash
# 用 pdftoppm 将论文每页转为图片
pdftoppm -r 300 papers/论文文件名.pdf papers/paper1_page
# 然后用图片查看器截图需要的图表区域
```

---

## 快速导航

| PPT 页码 | 推荐素材 | 来源 | 是否已复制 |
|----------|---------|------|-----------|
| 16 | 四旋翼 Micro-Doppler 谱图 | figures/micro_doppler_quadrotor_spectrogram.png | ✓ p16_quadrotor_microdoppler_spectrogram.png |
| 16 | UAV vs 噪声对比 | figures/micro_doppler_uav_vs_noise.png | ✓ p16_uav_detection_contrast.png |
| 17 | 论文一系统图 | 论文一 PDF 手动提取 | ✗ 需手动提取 |
| 18 | 论文一方法流程图 | 论文一 PDF 手动提取 | ✗ 需手动提取 |
| 19 | 论文一结果图表 | 论文一 PDF 手动提取 | ✗ 需手动提取 |
| 20 | 集成管线总览 | figures/integration_summary.png | ✓ p20_integration_pipeline_summary.png |
| 20 | 标准 Doppler 基线 | figures/baseline_doppler_etoile.png | ✓ p20_baseline_doppler_validation.png |
| 21 | 单散射点正弦调制 | figures/micro_doppler_single_scatterer.png | ✓ p21_single_scatterer_sinusoidal.png |
| 21 | 四旋翼谱图 | figures/micro_doppler_quadrotor_spectrogram.png | ✓ p21_quadrotor_spectrogram_full.png |
| 21 | 调制频率谐波 | figures/micro_doppler_modulation_frequency.png | ✓ p21_modulation_frequency_harmonics.png |
| 21 | 静止vs移动 | figures/micro_doppler_stationary_vs_moving.png | ✓ p21_stationary_vs_moving_uav.png |
| 22 | 项目总结 | figures/integration_summary.png | ✓ p22_project_integration_summary.png |
| 22 | 标准vs微多普勒 | figures/micro_doppler_vs_standard_doppler.png | ✓ p22_standard_vs_microdoppler_comparison.png |
