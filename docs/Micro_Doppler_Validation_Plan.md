# Micro-Doppler 集成到 Sionna RT —— 三步仿真验证计划

## 核心思路

Micro-Doppler 可以**不修改 Sionna 射线追踪引擎**，而是在 CIR 生成阶段给每条路径系数叠加正弦相位调制：

```
标准 Doppler：  a_i(t) = a_i * e^{-j2πfτ_i} * e^{j2πf_Δ t}           ← 线性相位斜坡
Micro-Doppler： a_i(t) = a_i * e^{-j2πfτ_i} * e^{jβ sin(2πf_rot t + φ₀)}  ← 正弦相位调制
```

其中调制指数 β = (4πR/λ) * cos(θ)，由叶片半径 R、旋转频率 f_rot、波长 λ 决定。

**关键洞察**：射线追踪负责静态场景的路径发现（a, τ, angles），旋转叶片的时变效应通过后处理叠加到路径系数上，不需要修改 RT 引擎。

---

## 第一步：Baseline — 标准 Doppler 验证

**目标**：用 Sionna RT 仿真移动 UE，验证能正确观察到标准 Doppler 频移，建立对照基线。

**具体步骤**：

1. 使用 `scene_dataset_create_paris.py` 的 Paris/Etoile 场景框架
2. 让 UE 沿固定方向匀速移动（如沿 x 轴 v = 10 m/s），BS 静止
3. 以固定时间间隔 Δt = 0.5 ms 多次调用 `path_solver()`，采集 CIR 序列（共 100 个时刻）
4. 对每条路径，从相邻时刻 CIR 的相位差提取 Doppler 频移：f̂_Δ = Δφ / (2π * Δt)
5. 验证 f̂_Δ ≈ v * fc / c（理论值对照）

**关键参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| fc | 28 GHz | 与现有数据集一致 |
| v_UE | 10 m/s | 沿 x 轴匀速 |
| Δt | 0.5 ms | 采样间隔 |
| N_snapshots | 100 | CIR 快照数 |
| 场景 | Paris (etoile) | 复用现有场景 |

**预期结果**：
- 直达径 (LoS) 的 Doppler 频移 ≈ 28e9 * 10 / 3e8 ≈ 933 Hz
- 各条反射径的 Doppler 频移由路径几何决定
- 相位随时间线性变化：φ(t) = φ₀ + 2π f_Δ t

**产出**：
- Doppler 频移仿真值 vs 理论值对比表
- 相位-时间线性关系图

---

## 第二步：核心验证 — 单旋转散射点 Micro-Doppler 仿真

**目标**：证明 Sionna RT 能捕获旋转运动产生的正弦相位调制，这是回答"能不能做"的关键实验。

**具体步骤**：

1. 建模单个旋转叶片上的散射点（位于旋转平面内）：
   - 散射点位置随时间变化：
     ```
     x(t) = x_body + R * cos(2π f_rot * t + φ₀)
     y(t) = y_body + R * sin(2π f_rot * t + φ₀)
     z(t) = z_body
     ```
   - R = 0.15 m（小型 UAV 叶片半径），f_rot = 100 Hz，φ₀ 为随机初始相位
   - UAV 本体位置 (x_body, y_body, z_body) 作为参考中心

2. 以采样率 fs = 2 kHz（满足 Nyquist：> 2 * f_rot）在 0.05 s 内采集 100 个 CIR 快照
   - 每个快照时刻更新 TX 位置为散射点的瞬时坐标
   - 调用 `path_solver()` 计算该时刻的路径

3. 从 CIR 序列中提取主路径的瞬时相位 φ(t)，做相位解缠（unwrap）
4. 对解缠后的相位做数值微分得到瞬时频率 f_inst(t) = dφ/dt / (2π)
5. 验证瞬时频率呈现正弦调制模式：f_inst(t) ≈ f_Δ + β * f_rot * cos(2π f_rot t + φ₀)

**关键参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| fc | 28 GHz | 毫米波频段 |
| R | 0.15 m | 小型 UAV 叶片半径 |
| f_rot | 100 Hz | 典型旋翼转速（6000 RPM） |
| fs | 2 kHz | CIR 采样率 |
| T_obs | 0.05 s | 观测时长（5 个旋转周期） |
| N_snapshots | 100 | 总快照数 |

**预期结果**：
- 调制指数 β = 4π * 0.15 / (3e8/28e9) ≈ 175.9 rad（考虑 cos(θ) 因子后约 50-150 rad）
- 瞬时频率在 f_Δ ± β*f_rot 范围内正弦波动
- 相位呈现 φ(t) = 2π f_Δ t + β sin(2π f_rot t) 的正弦调制形态

**产出**：
- 散射点旋转轨迹的 3D 可视化图
- 瞬时相位 φ(t) 的时间序列图（展示正弦调制模式）
- 瞬时频率 f_inst(t) 的时间序列图（展示周期性波动）
- 与标准 Doppler（线性相位）的并排对比图

---

## 第三步：应用验证 — 多旋翼 UAV Micro-Doppler 谱图与检测

**目标**：从仿真数据生成 Micro-Doppler 时频谱图，展示其可用于 UAV 检测/分类，回答导师"这个方向能不能做"。

**具体步骤**：

1. 扩展为完整四旋翼 UAV 模型：
   - 4 个旋翼臂（quadcopter），每个旋翼 2 个叶片 → 共 8 个散射点
   - 各旋翼位于正方形四角（对角距 0.35 m），不同初始相位
   - UAV 本体（机身）作为中心反射点，贡献常数 Doppler
   - 可选：加入 UAV 整体平移运动（如 v_body = 5 m/s）

2. 叠加多条路径贡献生成完整 CIR：
   ```
   h(t, τ) = Σ_k a_k(t) * δ(τ - τ_k(t))
   a_k(t) = A_k * exp(-j 2π fc τ_k(t))
   ```
   其中 τ_k(t) 是第 k 个散射点到 RX 的时变延迟

3. 对 CIR 序列做 STFT（短时傅里叶变换）：
   - 窗长 N_win = 128，重叠 75%
   - 频率分辨率 Δf = fs / N_win ≈ 15.6 Hz
   - 生成 Micro-Doppler 时频谱图

4. 分析谱图特征：
   - 零频附近的机身 Doppler 分量
   - 正负对称的旋转叶片 Doppler 边带（间距 = f_rot = 100 Hz）
   - 叶片数量与调制周期的对应关系（8 叶片 → 8*f_rot 的调制频率）

5. （加分项）添加基线检测算法：
   - 从谱图中提取 Micro-Doppler 特征（边带间距、带宽）
   - 与无 UAV 场景的谱图对比，证明可检测性

**关键参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| fc | 28 GHz | 载频 |
| R_blade | 0.15 m | 叶片半径 |
| f_rot | 100 Hz | 旋翼转速 |
| N_rotors | 4 | 旋翼数量 |
| N_blades | 8 | 总叶片数 |
| fs | 4 kHz | 提高采样率以捕获更高频调制 |
| T_obs | 0.1 s | 更长观测时长 |
| N_win | 128 | STFT 窗长 |
| v_body | 5 m/s | UAV 整体平移速度（可选） |

**预期结果**：
- Micro-Doppler 谱图中清晰可见周期性调制模式
- 边带间距 = f_rot = 100 Hz，与旋翼转速对应
- 最大 Doppler 偏移 ≈ (2 * v_tip) / λ ≈ (2 * 2π * 0.15 * 100) / 0.0107 ≈ 17.6 kHz
- 谱图呈现类似经典论文中的 "helicopter signature" 图案

**产出**：
- 多旋翼 UAV 模型的 Micro-Doppler 时频谱图（核心可视化）
- 有无 UAV 场景的谱图对比
- 调制频率 vs 叶片参数的关系验证表
- （可选）基于谱图特征的 UAV 检测 ROC 曲线

---

## 计划与三个核心问题的对应关系

| 师兄/导师的问题 | 对应步骤 | 回答方式 |
|----------------|---------|---------|
| **公式层面区别**：标准 Doppler vs Micro-Doppler 在计算上有什么不同？ | 第一步 → 第二步 | 第一步展示线性相位 = 常数 Doppler，第二步展示正弦调制 = 时变 Doppler，通过仿真数据直观对比 |
| **代码层面定位**：区别在 Sionna 代码中对应哪个位置？ | 第二步 | 不改 RT 引擎；改动在 CIR 生成时叠加时变相位调制：`a_k(t) = a_k * exp(-j2πfc * τ_k) * exp(jβ sin(2πf_rot t))`。整个流程中 RT 只调用一次处理静态场景，时变调制在 CIR 后处理阶段 |
| **可行性判断**：能不能做？难度多大？ | 第三步 | 第三步谱图直接证明可行性；改动范围限于后处理层，不涉及 RT 核心。工作量约 1-2 周（代码改动约 200 行，主要在新增 `MicroDopplerModulator` 类） |

---

## 环境配置（已验证通过 ✅）

| 组件 | 版本 | 状态 |
|------|------|------|
| Python | 3.11.15 | conda 环境 `sionna_rt` |
| Sionna (基础包) | 2.0.1 | ✅ 可导入 |
| Sionna RT | 2.0.1 | ✅ 可导入，场景加载/路径求解正常 |
| TensorFlow | 2.20.0 | ✅ GPU 可见 (RTX 4090) |
| Keras | 3.14.1 | ✅ |
| Mitsuba | 3.8.0 | ✅ CUDA 变体可用 (`cuda_ad_rgb` 等) |
| Dr.Jit | 1.3.1 | ✅ |
| PyTorch | 2.11.0 | ✅ (sionna 依赖) |
| CUDA Toolkit | 13.0.2 | ✅ |
| GPU | NVIDIA RTX 4090 (24 GB) | ✅ RT 核心可用于加速 |
| CUDA Driver | 12.8 | ✅ |
| NumPy | 2.4.4 | ✅ |
| SciPy | 1.17.1 | ✅ |

**激活环境**：
```bash
source /home/zfh/miniconda3/etc/profile.d/conda.sh && conda activate sionna_rt
```

---

## 现有可复用代码

| 文件 | 可复用部分 |
|------|-----------|
| `scene_dataset/scene_dataset_create_paris.py` | 场景构建、循环采样 CIR 的框架 |
| `tools/cir_to_cfr/dataset_CIR_to_CFR.py` | CIR 加载和 CFR 重建 |
| `tools/cir_to_cfr/Hgt_transform_to_HLS.py` | 准静态假设下的信道估计流程（可对比动态场景） |
| `RT_tutorial/sionna-rt/tutorials/Mobility.ipynb` | Sionna RT 移动性仿真示例 |
