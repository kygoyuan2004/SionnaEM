# Micro-Doppler 集成 Sionna RT — 后续三步计划（第四～六步）

## 背景

前三步已完成 Micro-Doppler 的物理验证（标准 Doppler → 单散射点 → 四旋翼谱图），证明了**不改 RT 引擎、在 CIR 后处理阶段叠加正弦相位调制**的技术路线可行。

后续三步的目标是：将验证脚本转化为**可复用的软件模块**，实际**接入 Sionna CIR 管道**，并完成与论文的**对标分析**，形成可以向何老师汇报的完整方案。

---

## 第四步：`MicroDopplerModulator` 核心模块实现

**目标**：实现一个独立的、可复用的 Micro-Doppler 调制器类，封装 UAV 参数和相位调制逻辑。

**产出文件**：`micro_doppler_modulator.py`

### 4.1 参数配置类 `UAVMicroDopplerConfig`

```python
@dataclass
class UAVMicroDopplerConfig:
    """四旋翼 UAV Micro-Doppler 仿真参数."""
    # 载波与波长
    carrier_freq: float = 28e9              # fc [Hz]

    # 旋翼几何
    n_rotors: int = 4                       # 旋翼数量
    n_blades_per_rotor: int = 2             # 每旋翼叶片数
    blade_radius: float = 0.15              # R [m]
    rotor_arm_length: float = 0.175         # 旋翼臂长（对角距 0.35 m）

    # 旋转动力学
    rotation_freq: float = 100.0            # f_rot [Hz]
    rotor_phases: tuple | None = None       # 各旋翼初始相位 [rad]，None 则随机

    # RCS / 幅度
    body_amplitude: float = 1.0             # 机身参考幅度
    blade_amplitude: float = 0.12           # 单叶片幅度（相对机身）

    # 雷达
    radar_position: tuple = (-40.0, 0.0, 10.0)  # [x, y, z] m
    radar_mode: str = "monostatic"          # "monostatic" | "bistatic"

    # UAV 运动
    body_position: tuple = (0.0, 0.0, 10.0)     # 初始位置 [x, y, z] m
    body_velocity: tuple = (0.0, 0.0, 0.0)      # 平移速度 [vx, vy, vz] m/s

    # 仿真
    sampling_rate: float = 50e3             # fs [Hz]
    obs_duration: float = 0.2               # T_obs [s]
    snr_db: float = 30.0                    # SNR [dB]
```

### 4.2 调制器类 `MicroDopplerModulator`

```python
class MicroDopplerModulator:
    """Micro-Doppler 正弦相位调制器.

    接收静态场景的路径参数（幅度 a、延迟 tau），按 UAV 模型叠加
    时变正弦相位调制，输出时变 CIR 序列。

    核心公式:
        a_k(t) = a_k * exp(j * β_k * sin(2π f_rot t + φ_k))

    其中 β_k = 4π R_blade / λ * cos(θ_k) 为第 k 个散射点的调制指数。
    """

    def __init__(self, config: UAVMicroDopplerConfig):
        ...

    @property
    def n_scatterers(self) -> int:
        """散射点总数 = 1 机身 + n_rotors × n_blades_per_rotor."""
        ...

    @property
    def beta(self) -> float:
        """理论最大调制指数 β = 4πR/λ."""
        ...

    def scatterer_positions(self, t: np.ndarray) -> np.ndarray:
        """计算所有散射点在时刻 t 的 3D 位置.

        Returns:
            np.ndarray, shape [n_scatterers, len(t), 3]
        """
        ...

    def modulate_cir(
        self,
        a_static: np.ndarray,      # CIR 幅度 [..., n_paths]
        tau_static: np.ndarray,    # CIR 延迟 [..., n_paths]
        t_vec: np.ndarray,         # 时间序列 [n_snapshots]
    ) -> tuple[np.ndarray, np.ndarray]:
        """对静态 CIR 叠加 Micro-Doppler 调制.

        对每条 RT 路径 k，将其视为从 UAV 中心发出的传播路径，
        叠加所有散射点在该路径上的时变相位贡献。

        Returns:
            a_dynamic: 时变 CIR 幅度 [n_snapshots, ..., n_paths]
            tau_dynamic: 时变 CIR 延迟 [n_snapshots, ..., n_paths]  (可选)
        """
        ...

    def generate_received_signal(
        self,
        t_vec: np.ndarray,
        add_noise: bool = True,
    ) -> np.ndarray:
        """生成接收信号（纯散射点模型，不含 RT 多径).

        基于 UAV 散射点模型的解析信号生成，用于快速验证和谱图生成。
        不依赖 Sionna RT 场景——用于独立测试。

        Returns:
            signal: 复基带信号 [len(t_vec)]
        """
        ...

    def stft_spectrogram(
        self,
        signal: np.ndarray,
        n_win: int = 2048,
        overlap_ratio: float = 0.75,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """对信号做 STFT，返回零中心的时频谱图.

        Returns:
            f: 频率轴 [Hz] (fftshift 后的 ±fs/2)
            t: 时间轴 [s]
            S_dB: 归一化功率谱 [dB], shape [freq_bins, time_bins]
        """
        ...

    def detect_harmonics(
        self,
        signal: np.ndarray,
        n_harmonics_max: int = 20,
    ) -> list[tuple[int, float]]:
        """从包络调制频谱检测 f_rot 谐波.

        Returns:
            List of (harmonic_order, frequency_Hz)
        """
        ...
```

### 4.3 关键方法详解

#### `scatterer_positions(t)` — 散射点几何

```
机身:  r_0(t) = r_body(0) + v_body * t

旋翼 j 中心:  r_rotor,j = r_body(t) + [±arm, ±arm, 0]
  其中 arm = rotor_arm_length, 四个旋翼分别位于正方形四角

叶片 k (属于旋翼 j, 相位 φ_k):
  r_k(t) = r_rotor,j + R * [cos(2π f_rot t + φ_k),
                              sin(2π f_rot t + φ_k),
                              0]
```

每个旋翼的 2 叶片相差 π（180° 对置）。

#### `modulate_cir(a_static, tau_static, t_vec)` — CIR 调制

这是核心方法，将静态 RT 路径参数转化为时变 CIR：

```
对于每条 RT 路径 p（从静态场景获取）:
  a_p, τ_p = 路径 p 的复幅度和延迟

  对每个散射点 k:
    距离 d_kp(t) = |r_k(t) - r_radar| + |r_tx - r_k(t)|  [双基地]
                 或 2 * |r_k(t) - r_radar|              [单基地]

    时变延迟: τ_kp(t) = d_kp(t) / c

    相位: φ_kp(t) = -2π fc * τ_kp(t)

    贡献: a_kp(t) = A_k * exp(j φ_kp(t))

  路径 p 的总贡献:
    a_p(t) = Σ_k a_kp(t)
    τ_p(t) ≈ τ_p (延迟变化相对较小，可近似为常数)
```

### 4.4 单元测试

| 测试项 | 验证方法 |
|--------|---------|
| `beta` 属性 | 对比手工计算值 4πR/λ |
| `n_scatterers` | 确认 = 1 + n_rotors * n_blades_per_rotor |
| 散射点轨迹半径 | 验证 |r_k(t) - r_rotor| ≡ R |
| 调制相位正弦性 | 对比 `generate_received_signal()` 相位与理论 sin 波形 |
| STFT 谱图对称性 | 静止 UAV 谱图左右对称于 f=0 |
| 机身 Doppler | 设定 v_body，提取谱图中心频率 ± 理论值 ±5% |
| 谐波间距 | 包络调制频谱峰间距 = f_rot ±10% |
| 与第三步结果一致性 | 相同参数下，谱图与 Step 3 输出一致 |

---

## 第五步：Sionna CIR 管道集成验证

**目标**：将 `MicroDopplerModulator` 实际接入 Sionna RT 的 CIR 生成管道，证明方案可以"嵌入"现有代码流。

**产出文件**：`micro_doppler_integration_demo.py`

### 5.1 集成流程

```
┌──────────────────────────────────────────────────────────┐
│ 1. 加载场景 & 静态 RT                                      │
│    scene = load_scene(etoile)                            │
│    scene.add(TX @ UAV_body_center)                       │
│    scene.add(RX @ radar_position)                        │
│    paths = solver(scene, ...)  ← 只调用一次！              │
│    a_static, tau_static = paths.cir(...)                 │
│                                                          │
│ 2. Micro-Doppler 调制（新增模块）                          │
│    config = UAVMicroDopplerConfig(...)                   │
│    modulator = MicroDopplerModulator(config)              │
│    a_dynamic, _ = modulator.modulate_cir(                │
│        a_static, tau_static, t_vec                       │
│    )                                                     │
│                                                          │
│ 3. 生成时变 CFR / 谱图                                    │
│    signal = Σ a_dynamic(t) * sinc(...)  ← 从 CIR 重构     │
│    S_dB, f, t = modulator.stft_spectrogram(signal)       │
│                                                          │
│ 4. 可视化 & 验证                                          │
│    plot spectrogram, compare with Step 3 reference        │
└──────────────────────────────────────────────────────────┘
```

### 5.2 验证场景

| 场景 | TX/RX 配置 | 验证内容 |
|------|-----------|---------|
| Paris etoile 自由空间 | BS [−40,0,10], UAV [0,0,10] | 基本集成验证 |
| Paris etoile 多径 | 同上 | 多径对 Micro-Doppler 的影响 |
| floor_wall 简化场景 | BS [−20,0,3], UAV [0,0,3] | 计算速度基准 |

### 5.3 对比验证

对每个场景，同时运行：

- **集成管线**（Sionna RT → static CIR → Modulator → dynamic CIR → spectrogram）
- **纯解析管线**（Step 3 的散射点模型 → spectrogram）

验证两者谱图特征一致（β、频偏、谐波间距）。

### 5.4 性能基准

| 指标 | 目标 |
|------|------|
| RT 求解耗时 | < 1s（一次调用） |
| `modulate_cir()` 耗时 | < 0.1s（10000 样本） |
| 总管线（RT + 调制 + STFT） | < 2s |
| 内存占用 | < 500 MB |

---

## 第六步：论文对标与最终汇报材料

**目标**：将本方法与两篇参考论文的方法进行定量对标，整理向何老师汇报的完整材料。

**产出文件**：`method_comparison.md` + 汇报 PPT 提纲

### 6.1 论文对标矩阵

#### 论文 A：*Micro-Doppler Signature Simulation of Multirotor UAVs Using Ray Tracing*

| 对比维度 | 论文方法 | 本方法 | 差异分析 |
|---------|---------|--------|---------|
| RT 引擎 | 自研 / 未公开 | Sionna RT (Mitsuba) | 本方法基于开源标准引擎 |
| 场景支持 | 自由空间 / 简单场景 | Paris etoile 等城市场景 | 本方法天然支持复杂环境 |
| Micro-Doppler 叠加 | 论文公式推导 | CIR 后处理 sin 调制 | 核心公式等价，实现路径不同 |
| 散射点模型 | PO 近似 | 点散射 + 恒定 RCS | 本方法 RCS 模型简化 |
| 叶片建模 | 连续叶片 | 端点散射 | PO 模型更精确（但更慢） |
| 计算效率 | — | RT 一次 + 调制后处理 | 本方法效率优势 |

#### 论文 B：*Micro-Doppler Signature-Based Detection, Classification and Localization of Small UAV*

| 对比维度 | 论文方法 | 本方法 |
|---------|---------|--------|
| 检测方法 | 谱图特征提取 | 可兼容（仿真数据可用于训练） |
| 分类特征 | 边带间距、带宽、闪烁频率 | 已提取（β、f_rot、800 Hz flash） |
| 定位能力 | — | 待扩展（需多基地雷达配置） |

### 6.2 定量对标实验

| 实验 | 输入 | 对比指标 | 预期结果 |
|------|------|---------|---------|
| 单旋翼谱图 | 同 β, f_rot | 边带间距、峰值频偏 | ±5% 以内一致 |
| 四旋翼谱图 | 论文参数 | 闪烁频率、谱图形状 | 定性一致 |
| 不同 β 值 | R ∈ {0.05, 0.1, 0.15, 0.2} m | β 与 R 的线性关系 | β ∝ R，R² > 0.999 |

### 6.3 何老师汇报材料

1. **一句话结论**：Sionna RT 可以集成 Micro-Doppler，改动仅在 CIR 后处理阶段（不改 RT 引擎），工作量约 1 周。

2. **关键可视化**（4 张图）：
   - 三步验证总览（标准 Doppler → 单点 Mod → 四旋翼谱图）
   - Micro-Doppler vs 标准 Doppler 并排对比
   - 集成管线架构图
   - UAV vs 噪声检测对比

3. **关键数据**（1 张表）：
   | 指标 | 值 |
   |------|-----|
   | β 仿真精度 | 0.00% 误差 |
   | CIR-几何相关性 | 1.000000 |
   | 边带检测成功率 | 100%（7 个谐波全部检测） |
   | RT 引擎修改量 | 0 行 |
   | 新增代码量 | ~200 行（MicroDopplerModulator） |
   | 总工作量 | 1 周（代码）+ 0.5 周（测试验证） |

---

## 时间线与依赖关系

```
第四步: MicroDopplerModulator
  ├── 4.1 Config dataclass        [2h]
  ├── 4.2 Scatterer geometry      [3h]
  ├── 4.3 modulate_cir()          [4h]  ← 核心
  ├── 4.4 signal generation       [2h]
  ├── 4.5 STFT / detection        [2h]
  └── 4.6 单元测试                [3h]
       ↓
第五步: CIR 管道集成              [依赖第四步完成]
  ├── 5.1 集成脚本                [3h]
  ├── 5.2 多场景验证              [3h]
  ├── 5.3 与 Step 3 对比          [2h]
  └── 5.4 性能基准                [1h]
       ↓
第六步: 论文对标 & 汇报           [依赖第四、五步完成]
  ├── 6.1 论文对照分析            [3h]
  ├── 6.2 定量对标实验            [2h]
  ├── 6.3 汇报材料整理            [3h]
  └── 6.4 最终报告更新            [2h]

总计: ~35 小时 (~4.5 工作日)
```

---

## 完成标准

三步全部完成后的最终状态：

- [ ] `micro_doppler_modulator.py` — 可复用的调制器模块（含 docstring + 类型注解）
- [ ] `micro_doppler_integration_demo.py` — 接入 Sionna CIR 的集成演示
- [ ] `method_comparison.md` — 与两篇论文的定量对标分析
- [ ] 更新 `three_step_validation_report.md` 为六步完整报告
- [ ] 所有测试通过，性能满足基准
- [ ] 汇报材料就绪，可向何老师展示
