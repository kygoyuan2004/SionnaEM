# SionnaEM 管线报告: Blender 场景 → Pathloss + CSI 张量

> 准备日期: 2026/05/27 | 用途: 何老师组会汇报

---

## 1. 管线架构总览

本管线实现从三维场景建模到无线电信道推理的端到端流程。

```
┌──────────────────────────────────────────────────────────────────┐
│                     Blender 三维场景建模                          │
│                                                                  │
│  ┌─────────────────────┐    ┌──────────────────────────────┐     │
│  │ mitsuba-blender 插件 │    │ Blender 原生 PLY 导出         │     │
│  │ → 直出 Mitsuba XML  │    │ → PLY + materials.json       │     │
│  └────────┬────────────┘    └────────────┬─────────────────┘     │
│           │                              │                       │
│           └──────────────┬───────────────┘                       │
│                          ▼                                       │
│              ┌──────────────────────┐                            │
│              │  Sionna RT 场景加载   │  ← load_scene(xml)        │
│              │  + 天线阵列配置       │  ← PlanarArray             │
│              │  + 基站 & 无人机部署  │  ← Transmitter/Receiver    │
│              └──────────┬───────────┘                            │
│                         ▼                                        │
│              ┌──────────────────────┐                            │
│              │  PathSolver 射线追踪  │  ← max_depth=5            │
│              │  → 多径传播路径求解   │                            │
│              └──────────┬───────────┘                            │
│                         ▼                                        │
│         ┌───────────────────────────────┐                        │
│         │        CSI 张量构建            │                        │
│         │                               │                        │
│         │  CIR: ã_p, τ_p, f_Δ, θ, φ    │                        │
│         │  CFR: H[k]= Σ_p ã_p·e^{-j2πf_kτ_p} │                 │
│         └───────────────┬───────────────┘                        │
│                         ▼                                        │
│         ┌───────────────────────────────┐                        │
│         │    导出指标 & 可视化           │                        │
│         │  · Pathloss 图 (热力图)        │                        │
│         │  · CSI 幅度 / 相位             │                        │
│         │  · Delay Spread / Angular Spread│                       │
│         │  · 3D 场景渲染 (pathloss 叠加) │                        │
│         │  · .npz 数据导出               │                        │
│         └───────────────────────────────┘                        │
└──────────────────────────────────────────────────────────────────┘
```

### 1.1 文件结构

```
pipeline/
├── step8_blender_scene_interface.py   # Blender 场景接口层
├── step9_pipeline_pathloss_csi.py     # 完整管线主脚本
├── variable_reference.md              # 变量完整参考文档
├── pipeline_report.md                 # 本报告
├── figures/                           # 输出图 (9 张)
│   ├── pipeline_pathloss_*.png
│   ├── pipeline_csi_magnitude_*.png
│   ├── pipeline_csi_phase_*.png
│   ├── pipeline_delay_spread_*.png
│   ├── pipeline_angular_spread_*.png
│   ├── pipeline_summary_4panel_*.png
│   ├── pipeline_scene_pathloss_*.png
│   ├── pipeline_scene_rays_*.png
│   └── pipeline_dual_band_comparison.png
└── data/                              # CSI 张量 (.npz)
    └── pipeline_csi_*.npz
```

---

## 2. 组件说明

### 2.1 Blender 场景接口 (step8)

`BlenderSceneSpec` 数据类定义了 Blender 导出与 Sionna RT 之间的契约。

**支持三种场景来源**:

| 优先级 | 方式 | 说明 |
|--------|------|------|
| 1 | Mitsuba XML 直出 | Blender 安装 mitsuba-blender 插件，直接导出 `.xml` → `load_scene()` |
| 2 | PLY 中转 | Blender 导出 PLY 网格 → `generate_scene_xml.py` 生成 XML |
| 3 | 程序化回退 | 代码自动生成四旋翼无人机 + 地面场景（PLY + XML） |

**Blender 导出设置**（PLY 中转方式）:
- Format: ASCII
- Forward: Y Forward, Up: Z Up
- 勾选: Apply Modifiers, Triangulate Faces
- 材质映射: 创建 `materials.json` 指定每个网格的 ITU 材料

### 2.2 管线脚本 (step9)

命令行接口:
```bash
cd pipeline
python step9_pipeline_pathloss_csi.py                    # 默认: 程序化, 双频段
python step9_pipeline_pathloss_csi.py --ofdm              # 启用 OFDM CFR
python step9_pipeline_pathloss_csi.py --freq 28           # 仅 28 GHz
python step9_pipeline_pathloss_csi.py --scene-xml scene.xml  # Mitsuba 直出
```

管线步骤:
1. 场景加载 → 2. PathSolver → 3. CIR 提取 → 4. CSI 张量 → 5. RadioMapSolver → 6. 3D 渲染 → 7. 9 张图 + .npz

---

## 3. 数学背景

### 3.1 信道脉冲响应 (CIR)

Sionna RT 通过射线追踪计算每条路径的复基带系数 ã_p 和时延 τ_p:

```
ã_p = a_p · exp(-j·2π·f_c·τ_p)

其中:
  a_p  = 通带路径系数 (包含反射/透射损耗)
  f_c  = 载波频率
  τ_p  = 传播时延 = d_p / c
```

CIR 输出 shape:
- `a_cpx`: `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps]`
- `tau`: `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]` [s]

### 3.2 信道频率响应 (CFR)

从 CIR 重建 OFDM 子载波上的频率响应:

```
H[k] = Σ_{p=1}^{P} ã_p · exp(-j·2π·f_k·τ_p)

其中:
  K     = 子载波数 (默认 1024)
  Δf    = 子载波间隔 = B/K = 120 kHz
  f_k   = (k - K/2) · Δf   (第 k 个子载波的基带频率偏移)
```

CFR shape: `[num_tx, num_rx, K]` complex64.

### 3.3 路径损耗 (Pathloss)

```
Per-path:    PL_p = -20·log10(|ã_p|)       [dB]
Combined:    PL = -10·log10(Σ_p |ã_p|²)    [dB]
Free-space:  PL_FS = 20·log10(4πd/λ)       [dB]
```

### 3.4 均方根时延扩展 (RMS Delay Spread)

```
τ̄    = Σ_p (P_p · τ_p) / Σ_p P_p           (功率加权平均时延)
τ̄²   = Σ_p (P_p · τ_p²) / Σ_p P_p          (功率加权均方时延)
σ_τ  = sqrt(Σ_p(P_p·τ_p²)/Σ_p P_p - τ̄²)      (RMS delay spread)
```

其中 P_p = |ã_p|² 为第 p 条路径的功率。

物理含义: σ_τ 越大，频率选择性衰落越严重，需要更复杂的均衡器。

### 3.5 角度扩展 (Angular Spread)

对 AoD/AoA 的四个角度方向 (θ_t, φ_t, θ_r, φ_r) 分别计算:

```
σ_θ = sqrt( Σ_p (P_p · (θ_p - θ̄)²) / Σ_p P_p )
```

角度扩展反映多径在空间域的分散程度，影响 MIMO 分集增益。

---

## 4. 双频段对比: 3.5 GHz vs 28 GHz

| 特性 | 3.5 GHz (Sub-6) | 28 GHz (mmWave) |
|------|----------------|-----------------|
| 波长 λ | 85.7 mm | 10.7 mm |
| 自由空间路径损耗 (d=20m) | ~69 dB | ~87 dB |
| 绕射能力 | 强 | 弱（接近光学传播） |
| 反射损耗 | 较低 | 较高 |
| 材料穿透 | 较好 | 差（易被遮挡） |
| 可用带宽 | ~100 MHz | ~400 MHz~1 GHz |
| 多普勒灵敏度 | 基准 | 8×（相同速度下多普勒频移 8 倍） |
| 微多普勒调制指数 β | ~175 rad (@ R=0.15m) | ~1400 rad (@ R=0.15m) |
| 建议 cell_size | [2.0, 2.0] m | [0.5~1.0, 0.5~1.0] m |

**关键差异**:
1. mmWave 路径损耗比 Sub-6 大约 18 dB
2. mmWave 对微多普勒更加敏感（β ∝ f_c）
3. Sub-6 适合广域覆盖，mmWave 适合高精度感知

---

## 5. Blender 使用指南

### 5.1 方案一: mitsuba-blender 插件直出（推荐）

1. 安装 mitsuba-blender 插件
2. 在 Blender 中建模场景（建筑物、无人机等）
3. 使用插件的导出功能直接导出 `.xml` 文件
4. 确保 XML 中的材质使用 ITU 无线电材料标准

```bash
python step9_pipeline_pathloss_csi.py --scene-xml /path/to/exported_scene.xml
```

### 5.2 方案二: PLY 导出中转

1. Blender → File → Export → Stanford PLY (.ply)
2. 设置: ASCII, Y Forward, Z Up, Triangulate Faces
3. 创建 `materials.json` 材质映射
4. 运行 `generate_scene_xml.py` 生成 XML

### 5.3 方案三: 程序化生成（无需 Blender）

```bash
python step9_pipeline_pathloss_csi.py                    # 自动程序化场景
```

---

## 6. 结果输出说明

### 6.1 生成图说明

| 图名 | 内容 | 用途 |
|------|------|------|
| `pipeline_pathloss_*.png` | per-BS + combined path gain 和 RSS 热力图 | 展示空间覆盖 |
| `pipeline_csi_magnitude_*.png` | 路径功率分布 + CFR 幅度 + 功率-时延散点 | CSI 幅度分析 |
| `pipeline_csi_phase_*.png` | 路径相位-时延 + CFR 相位 | CSI 相位分析 |
| `pipeline_delay_spread_*.png` | 功率延迟分布 + per-BS RMS-DS | 时延扩展分析 |
| `pipeline_angular_spread_*.png` | AoD/AoA 角度分布极坐标图 | 角度扩展分析 |
| `pipeline_summary_4panel_*.png` | 四合一: pathloss + PDP + CSI + AoA | 一页总览 |
| `pipeline_scene_pathloss_*.png` | 3D 场景 + pathloss 叠加渲染 | 场景可视化 |
| `pipeline_scene_rays_*.png` | 3D 场景 + 射线路径可视化 | 传播路径可视化 |
| `pipeline_dual_band_comparison.png` | 3.5 vs 28 GHz 双频对比 | 频段对比 |

### 6.2 .npz 数据文件

每个频段生成一个 `.npz` 文件，包含:
- `a_cpx`, `tau`: CIR 系数和时延
- `path_gain`, `path_gain_db`, `rss`: 无线电地图数据
- 各导出指标 (pathloss, delay spread 等)

---

## 7. 项目变量速查

| 类别 | 关键变量 | Shape | 单位 |
|------|---------|-------|------|
| 场景 | `scene.frequency` | scalar | Hz |
| 路径 | `paths.a` | `[num_rx, num_tx, num_paths]` | — |
| 时延 | `tau` | `[... num_paths]` | s |
| 多普勒 | `paths.doppler` | `[... num_paths]` | Hz |
| CIR | `a_cpx` | `[..., num_paths, num_time_steps]` | — |
| CFR | `H_cfr` | `[num_tx, num_rx, K]` | — |
| 路径增益 | `path_gain` | `[num_tx, cells_y, cells_x]` | 无量纲 |
| RSS | `rss` | `[num_tx, cells_y, cells_x]` | dBm |
| RMS-DS | σ_τ | scalar | s |
| 角度扩展 | σ_θ | scalar | rad |

> 完整变量文档见 `variable_reference.md`

---

## 8. 下一步工作

1. **动态场景**: 集成 step7 的无人机轨迹，在时间序列上计算 CSI
2. **ML 数据集**: 批量生成不同无人机位置/姿态的 CSI 数据用于训练
3. **天线阵列**: 扩展为多天线 MIMO 配置
4. **Blender 复杂场景**: 使用 mitsuba-blender 插件导入城市级场景
