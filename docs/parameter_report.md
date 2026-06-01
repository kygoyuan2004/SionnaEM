# SionnaEM 项目参数说明报告

> 完整参数参考手册 — 适用于组会汇报、论文写作与技术交流
>
> 覆盖范围：UAV 几何与运动 / 旋翼参数 / 雷达通信系统 / Micro-Doppler 理论 /
> Sionna RT 射线追踪 / STFT 时频分析 / EMD 分解 / PCA 降维 /
> LSTM 分类定位 / 实验数据集与评价指标 / Blender 场景建模

---

## 目录

1. [UAV 几何与运动参数](#1-uav-几何与运动参数)
2. [旋翼 / 螺旋桨参数](#2-旋翼--螺旋桨参数)
3. [雷达 / 通信系统参数](#3-雷达--通信系统参数)
4. [Micro-Doppler 理论公式参数](#4-micro-doppler-理论公式参数)
5. [Sionna RT / Ray Tracing 仿真参数](#5-sionna-rt--ray-tracing-仿真参数)
6. [STFT 时频分析参数](#6-stft-时频分析参数)
7. [EMD 与 IMF 相关参数](#7-emd-与-imf-相关参数)
8. [PCA 降维参数](#8-pca-降维参数)
9. [LSTM 分类与定位模型参数](#9-lstm-分类与定位模型参数)
10. [实验数据集与评价指标参数](#10-实验数据集与评价指标参数)
11. [Blender 场景建模参数](#11-blender-场景建模参数)
12. [易混淆参数对比表](#12-易混淆参数对比表)
13. [关键公式汇总](#13-关键公式汇总)
14. [参数速查索引](#14-参数速查索引)

---

## 1. UAV 几何与运动参数

### 1.1 机身几何参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 机身尺寸 | — | 无人机机身长宽高 | Body box dimensions | `step6_static_drone_scene.py` | 四旋翼中心机身几何大小 | m | 0.30×0.30×0.06 (项目); 视机型而定 | 决定机身 RCS (雷达散射截面)；机身反射信号通常占主导，远强于叶片回波 |
| 机身幅度 | `A_body` | 机身散射体幅度 | Body scatterer amplitude weight | `micro_doppler_modulator.py` | 机身相对 RCS 权重 | 无量纲 | 1.0 (项目); 0.18–0.25 (论文复现) | 决定了平动多普勒分量在谱图中的功率占比；值越大，机身谱线越强 |
| 旋翼臂长 | `rotor_arm_length` / `R_arm` | 旋翼臂半对角长度 | Rotor arm half-diagonal length | `micro_doppler_modulator.py` | 机身中心到旋翼转轴的距离 | m | 0.175 (项目) | 决定 4 个旋翼的空间分布；影响各旋翼散射体的相位差 |
| 旋翼臂半径 | `arm_radius` | 臂圆柱半径 | Arm cylinder radius | `step6_static_drone_scene.py` | 旋翼支撑臂结构粗细 | m | 0.015 (项目) | 仅影响 Sionna RT 的几何建模精度，对微多普勒结果无直接影响 |

### 1.2 无人机运动参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 平动速度 | **V_body** | 机身平移速度矢量 | Body translation velocity | `micro_doppler_modulator.py` `step3_quadrotor_spectrogram.py` | UAV 整体运动的速度矢量 | m/s | [5,0,0] (项目); [2,1,0] (step7); 0 (论文 TGRS) | 产生恒定的平动多普勒频移 f_D；在谱图中体现为整体谱线的水平偏移；不影响微多普勒带宽 |
| 初始位置 | **r_0** / `body_position` | UAV 初始位置 | Initial body position | `UAVMicroDopplerConfig` | UAV 在全局坐标系中的起始坐标 | m (x,y,z) | [0, 0, 10] (项目 step3); [0, 0, 3] (step6) | 决定 UAV 与雷达的距离；距离越远路径损耗越大，SNR 越低 |
| 欧拉角 | (ψ, θ, φ) | 偏航/俯仰/滚转角 | Yaw / Pitch / Roll | 论文 Eq.(5) | UAV 机体姿态的三维旋转角度 | rad 或 deg | (0,0,0) 典型值; 实验中可变化 | Yaw 仅改变初始相位；Pitch/Roll 改变微多普勒最大频偏（通过改变 cos(θ) 中的观测角） |
| 观测角 / 纵横角 | **θ_a** / **β_aspect** | 雷达视线与旋翼运动平面的投影角 / 论文中的纵横角 | Aspect angle | TGRS Eq.(4); 项目调制指数公式 | 决定叶片切向速度在雷达视线方向上的投影 | rad 或 deg | 0 ~ π | **核心参数**：速度投影越大，微多普勒越强；当 LOS 沿旋翼轴线观察时叶片径向速度近似为 0，微多普勒消失；当 LOS 位于旋翼盘平面内时峰值最大 |
| 双基地角 | **φ_b** | 双基地角 (TX-目标-RX 夹角) | Bistatic angle | TGRS Eq.(4), 双基地几何公式 | 发射→目标→接收两条路径的夹角 | rad 或 deg | 0–180° | 双基地雷达中可观测频移含 cos(φ_b/2) 因子；单基地时 φ_b=0, cos(0)=1；φ_b=180° (前向散射) 时 β→0 |
| 初始相位 | **φ_0** / `rotor_phases` | 旋翼初始旋转角度 | Initial rotor phase | `UAVMicroDopplerConfig.rotor_phases` | t=0 时刻叶片的起始角位置 | rad | 随机 [0, 2π) (项目); 种子 12345 | 确定正弦调制的起始点；不影响谱图整体结构，仅造成时间轴上的相位偏移 |

### 1.3 关键运动参数对结果的影响

**平动多普勒 vs 微多普勒的分离**:

- 平动速度 V_body 产生恒定的多普勒频移 `f_D = -2f_c·V_radial / c`
- 旋翼转动产生时变的正弦微多普勒 `f_mD(t) = β_m·f_rot·cos(2πf_rot·t + φ_0)`，其中 `β_m` 是相位调制指数，不要和 TGRS Eq.(4) 中表示 aspect angle 的 `β` 混淆
- 在 STFT 谱图中，平动分量是一条水平线，微多普勒分量是以平动分量为中心的正弦波动
- **汇报解释**："无人机整体飞行产生固定多普勒频移，旋翼旋转叠加了周期性的微多普勒调制"

---

## 2. 旋翼 / 螺旋桨参数

### 2.1 几何参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 叶片半径 | **R** / `blade_radius` / `L_b` | 叶片旋转半径 (半桨长) | Blade radius / half propeller length | `UAVMicroDopplerConfig.blade_radius`; TGRS Eq.(4); ICCT Table I | 旋翼转轴到叶片尖端的距离 | m | 0.15 (项目); 10.55 cm (ICCT Table I); TGRS 使用 DJI Phantom 3 碳纤维桨但未在正文表格给出具体桨长 | **核心参数**：β_m ∝ R，峰值频偏 f_dev_peak ∝ R；R 越大，微多普勒带宽越宽 |
| 叶片数 | **N_blades** / `n_blades_per_rotor` | 每旋翼叶片数 | Blades per rotor | `UAVMicroDopplerConfig.n_blades_per_rotor` | 单个旋翼上的叶片数量 | 无量纲 | 2 (项目及大部分无人机) | 叶片闪烁频率 f_flash = N_blades × f_rot；偶数叶片对称配置时偶数次谐波占主导 |
| 旋翼数 | **N_rotors** / `n_rotors` | 旋翼数量 | Number of rotors | `UAVMicroDopplerConfig.n_rotors` | 无人机上的旋翼/电机总数 | 无量纲 | 4 (四旋翼); 1 (直升机) | 旋翼越多 MDS 越复杂；四旋翼的 MDS 比单旋翼更"模糊"（多个正弦调制的叠加） |
| 总叶片数 | `n_blades` | 总散射体数量 (仅叶片) | Total blade scatterer count | `UAVMicroDopplerConfig.n_blades` | N_rotors × N_blades_per_rotor | 无量纲 | 8 (=4×2, 项目) | 信号中微多普勒散射体的总数量；叶片越多谱图越复杂 |

### 2.2 运动参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 旋转频率 | **f_rot** / `rotation_freq` | 旋翼旋转频率 (每秒转数) | Rotation frequency (RPS) | `UAVMicroDopplerConfig.rotation_freq` | 旋翼每秒的转动圈数 | Hz (=RPS) | 100 (项目，对应6000 RPM); 50–200 (典型无人机); 159.15 (9550 RPM, 论文 ICCT) | **核心参数**：决定谱图中边带间隔（= f_rot）；f_rot 越大，边带越分散；f_dev_peak ∝ f_rot |
| 角速度 | **ω_rot** / `omega_rot` | 旋翼旋转角速度 | Angular rotation frequency | 计算 `2π·f_rot` | 旋翼转动的角速度 | rad/s | 2π×100 ≈ 628 (项目) | 用于叶片尖端线速度 v_tip = ω_rot × R；决定瞬时径向速度分量 |
| 叶片尖端线速度 | **v_tip** | 叶片尖端线速度 | Blade tip linear velocity | 计算 `2π·f_rot·R` | 叶片最外端点的切向速度 | m/s | ≈94.2 (R=0.15, f_rot=100) | 决定了最大微多普勒频偏的理论上限：f_max = 2f_c·v_tip/c |
| 旋转方向 | `direction` | 旋转方向 | Rotation direction | `paper_fig2_style_mds.py` Rotor 类 | +1 表示顺时针，-1 表示逆时针 | 无量纲 | +1 或 -1 | 邻接旋翼反转可部分抵消扭矩效应；不影响微多普勒谱的功率分布 |

### 2.3 散射特性参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 叶片散射幅度 | `A_blade` / `blade_amplitude` | 叶片尖端散射幅度 | Blade scatterer amplitude | `UAVMicroDopplerConfig.blade_amplitude` | 每个叶片散射体相对于机身的 RCS 权重 | 无量纲 | 0.12 (项目) | 决定微多普勒信号在总接收信号中的相对强度；A_blade 太小则微多普勒淹没在噪声中 |
| 叶片闪烁权重 | `blade_flash_weight` | 叶片闪烁调制深度 | Blade flash modulation weight | `paper_fig2_style_mds.py` | 叶片旋转到面向雷达时 RCS 的周期增强因子 | 无量纲 | 0.18–0.22 (论文复现) | 在谱图中产生频率为 N_blades×f_rot 的周期性幅度调制，形成"闪烁"特征 |
| 叶片材料 | — | 螺旋桨材料 | Propeller material | 论文 TGRS 表1 | 叶片构成材料 | — | 碳纤维 (DJI Phantom 3); 木质 (论文 ICCT) | 影响叶片的实际 RCS 值和散射方向图；金属/碳纤维 RCS 更高 |

### 2.4 汇报重点

- **R (叶片半径) 和 f_rot (旋转频率) 是两个最关键的参数**：它们共同决定了微多普勒的相位调制指数 β_m 和峰值频偏 f_dev_peak
- **四旋翼 vs 单旋翼 (直升机) 的 MDS 差异**：四旋翼有 4 个旋翼（8 个叶片），各自有略微不同的转速和随机初始相位，导致 MDS 更为"模糊"；单旋翼的 MDS 更干净，更适合用于定位
- **叶片闪烁效应**：当叶片旋转到正对雷达的方向时，RCS 瞬间增强，形成周期性幅度尖峰，频率 = N_blades × f_rot

---

## 3. 雷达 / 通信系统参数

### 3.1 频率参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 载波频率 | **f_c** / `carrier_freq` | 载波频率 | Carrier frequency | `UAVMicroDopplerConfig.carrier_freq` | 雷达发射信号的中心频率 | Hz | 3.5e9 (Sub-6), 28e9 (mmWave), 915e6 (论文 TGRS) | **核心参数**：β_m ∝ f_c，mmWave 的微多普勒比 Sub-6 强 8 倍；但 mmWave 路径损耗更大，是核心 trade-off |
| 波长 | **λ** / `wavelength` | 波长 | Wavelength | 计算 `c/f_c` | 电磁波的空间周期 | m | 10.7 mm (28 GHz); 85.7 mm (3.5 GHz); 32.8 cm (915 MHz) | 与 β_m 成反比；λ 越小 β_m 越大；X 波段到毫米波的变化可让调制指数增加 20+ 倍 |
| 带宽 | **B** / `bandwidth` | 系统带宽 | System bandwidth | Scene 配置; `--bandwidth` | 发射信号的总频率范围 | Hz | 122.88e6 (5G NR 参考); 100e6 (Sub-6); 400e6+ (mmWave) | 影响距离分辨率 Δr = c/(2B)；宽带宽可分辨单个散射体（扩展目标效应） |
| 采样率 | **f_s** / `sampling_rate` | 基带采样率 | Baseband sampling rate | `UAVMicroDopplerConfig.sampling_rate` | ADC 采样频率 | Hz | 50000 (项目 step3+); 32000 (论文 TGRS); 2000 (项目 step1-2) | 必须满足 Nyquist 条件：f_s > 2×f_dev_peak；采样不足会导致微多普勒混叠 |
| 子载波间隔 | **Δf** / `scs` | OFDM 子载波间隔 | Subcarrier spacing | `tools/cir_to_cfr/` | OFDM 相邻子载波间的频率间隔 | Hz | 120e3 (=122.88e6/1024) | 影响最大可测量多普勒范围和符号间干扰 |
| 子载波数 | **K** | OFDM 子载波数 | Number of subcarriers | `--num-subcarriers` | OFDM 子载波总数 | 无量纲 | 1024 (默认) | K 越大频率分辨率越细；CFR 张量 `[num_tx, num_rx, K]` |

### 3.2 功率与噪声参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 发射功率 | `P_TX` / `power_dbm` | 发射功率 | Transmit power | Transmitter 配置 | 基站/雷达的输出功率 | dBm | 44 (标准基站约25W); 0 (微多普勒测试) | 影响接收 SNR；研发模式下低功率避免接收机饱和 |
| 信噪比 | **SNR** / `snr_db` | 信噪比 | Signal-to-Noise Ratio | `UAVMicroDopplerConfig.snr_db` | 信号功率与噪声功率之比 | dB | 30 (项目默认); 18–22 (论文复现) | SNR 越高 MDS 越清晰；低 SNR 下弱叶片回波被噪声淹没；论文 TGRS 中噪声场景 SNR 更低 |
| 噪声功率 | — | 热噪声功率 | Thermal noise power | `scene.thermal_noise_power` | k·T·B | W | k·293·B | 随带宽增加而增加；mmWave 大带宽下需考虑 |

### 3.3 天线参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 天线阵列行数 | `num_rows` | 天线阵列行数 | Antenna array rows | PlanarArray | 天线阵列的行方向的单元数 | 无量纲 | 1 (项目微多普勒); 32 (大规模 MIMO) | 多天线可提供阵列增益和波束赋形 |
| 天线阵列列数 | `num_cols` | 天线阵列列数 | Antenna array columns | PlanarArray | 天线阵列的列方向的单元数 | 无量纲 | 1 (项目微多普勒); 32 (大规模 MIMO) | 同上 |
| 天线方向图 | `pattern` | 天线方向图类型 | Antenna radiation pattern | PlanarArray | 单个阵元的辐射方向图 | — | `"iso"` (全向), `"tr38901"` (3GPP标准), `"dipole"` | iso 各方向均匀；tr38901 有方向性增益约 8 dBi |
| 极化方式 | `polarization` | 极化方式 | Polarization | PlanarArray | 天线极化方向 | — | `"V"` (垂直), `"H"` (水平), `"VH"` (双极化) | 对微多普勒无显著影响；影响地物反射的极化耦合 |
| 天线增益 | — | 天线增益 | Antenna gain | 论文 TGRS | 天线定向辐射能力 | dBi | 6.1 (论文 TGRS 贴片天线) | 提高增益可增加 SNR，但减小覆盖范围 |
| 波束宽度 (水平) | — | 水平 3dB 波束宽度 | Horizontal beamwidth | 论文 TGRS | 半功率波束宽度 | deg | 122° (论文 TGRS) | 目标越偏离主瓣，信号越弱 |
| 波束宽度 (垂直) | — | 垂直 3dB 波束宽度 | Vertical beamwidth | 论文 TGRS | 半功率波束宽度 | deg | 68° (论文 TGRS) | 同上 |

### 3.4 雷达模式与几何

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 雷达模式 | `radar_mode` | 单基地/双基地雷达 | Monostatic / Bistatic radar | `UAVMicroDopplerConfig.radar_mode`; TGRS Eq.(4) | 发射与接收天线是否在同一位置 | — | `"monostatic"` (项目默认); `"bistatic"` (TGRS 实验) | 单基地为往返路径，常见公式含 2 倍径向速度/4π 相位项；双基地频移还受 `cos(φ_b/2)` 影响。不能简单脱离几何说“永远翻倍” |
| 发射机位置 | `tx_position` | 发射天线位置 | Transmitter position | Transmitter 配置 | 全局坐标系中 TX 的三维坐标 | m (x,y,z) | [-40, 0, 10] (项目微多普勒) | 与 RX 位置共同决定双基地几何 |
| 接收机位置 | `rx_position` / `radar_position` | 接收天线位置 | Receiver / Radar position | Receiver 配置 | 全局坐标系中 RX 的三维坐标 | m (x,y,z) | [-40, 0, 10] (项目微多普勒); 多 BS 部署 (step6) | 决定雷达视线方向 (LOS)，影响 cos(θ) 中的观测角 |
| 波形类型 | — | 发射波形 | Waveform type | 论文 ICCT | 雷达发射信号体制 | — | CW 单音 (论文 TGRS); OFDM (论文 ICCT/项目) | CW 无双程时延信息；OFDM 可实现距离-多普勒联合处理 |

### 3.5 汇报重点

- **f_c 是微多普勒应用的核心 trade-off 变量**：频率越高 β_m 越大 → 微多普勒特征越明显 → 检测/分类/定位性能越好；但频率越高路径损耗越大 → 作用距离越短
- **单基地 vs 双基地**：单基地可看作收发同址的往返相位变化；双基地由 TX-target-RX 几何共同决定，并通过 `cos(φ_b/2)` 改变可观测速度投影。汇报时强调“几何投影”比只记公式因子更稳。
- **mmWave (28 GHz) 相比 Sub-6 GHz (3.5 GHz) 对微多普勒的灵敏度提升约为 8 倍**

---

## 4. Micro-Doppler 理论公式参数

这是本项目的核心理论框架。以下参数均出自微多普勒物理模型和信号模型。

### 4.1 核心公式参数

| 参数 | 符号 | 中文含义 | 英文含义 | 公式位置 | 物理意义 | 单位 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|---------|---------|------|---------|------------------|
| 调制指数 | **β_m** | 微多普勒相位调制指数 | Micro-Doppler phase modulation index | 项目: `β_m = 4π·R·cos(θ_proj) / λ` (单基地近似); 双基地需乘几何投影项 `cos(φ_b/2)` | 正弦相位调制的峰值幅度 | rad | 176.1 (28 GHz, R=0.15, θ_proj≈0); 22.0 (3.5 GHz); 5.75 (915 MHz, 同 R 和单基地几何) | **最关键参数**：β_m 决定了可见边带数量；β_m 越大边带越多，频谱越宽。TGRS 使用低频 915 MHz，因此 MDS 明显更模糊 |
| 峰值频偏 | **f_dev_peak** | 微多普勒峰值频率偏移 | Peak frequency deviation | `f_dev_peak = β_m × f_rot` | 瞬时频率的最大偏移量 | Hz | ±17.6 kHz (28 GHz, f_rot=100); ±2.2 kHz (3.5 GHz); ±575 Hz (915 MHz, R=0.15, f_rot=100, 单基地参考) | 定义了微多普勒谱的总带宽 ≈ 2 × f_dev_peak；超出此范围为噪声 |
| 平动多普勒 | **f_D** | 平动多普勒频移 | Translational Doppler shift | `f_D = -(2f_c/c)·(V·n)` | UAV 整体运动产生的恒定多普勒频移 | Hz | -934 (v_x=5 m/s, 28 GHz); -467 (单基地); 依赖于 v_radial | 在谱图中为一条水平线；与微多普勒叠加；f_mD(t) 以此为中心波动 |
| 微多普勒瞬时频率 | **f_mD(t)** | 瞬时微多普勒频率 | Instantaneous micro-Doppler frequency | `f_mD(t) = β_m·f_rot·cos(2πf_rot·t + φ_0)`；TGRS Eq.(4): `2f|v(n)|cos(φ_b/2)cos(β_aspect)/v_c` | 旋翼在 t 时刻产生的频率偏移 | Hz | 时变正弦，范围 [-f_dev_peak, +f_dev_peak] | MDS 的核心内容：在时频面上画出一条或多条正弦曲线 |
| 贝塞尔系数 | **J_n(β_m)** | n 阶第一类贝塞尔函数 | Bessel function of the first kind | `s(t) = Σ_n J_n(β_m)·exp(j·2π·(f_c+f_D+n·f_rot)·t)` | 第 n 个边带的复幅度 | 无量纲 | J_n(β_m) 随 n 增大衰减 | 决定每个边带的功率：n=0 为载波线，n=±1 为一阶边带… |
| 叶片闪烁频率 | **f_flash** | 叶片闪烁频率 | Blade flash frequency | `f_flash = N_blades × f_rot` | 叶片 RCS 周期性增强的频率 | Hz | 200 (N_blades=2, f_rot=100); 800 (N_blades=4, f_rot=200) | 在谱图中表现为周期性的幅度调制，频率 = f_flash |

### 4.2 信号模型参数

| 参数 | 符号 | 中文含义 | 英文含义 | 公式位置 | 物理意义 | 单位 | 典型取值 |
|------|------|---------|---------|---------|---------|------|---------|
| 接收信号 | **s(t)** | 基带接收信号 | Received baseband signal | `s(t) = Σ_k A_k·exp(-j·4π·d_k(t)/λ) + n(t)` | 所有散射路径回波的相干叠加 | 复数 | 复包络 |
| 瞬时相位 | **φ(t)** | 瞬时信号相位 | Instantaneous phase | `φ(t) = 2πf_D·t + β_m·sin(2πf_rot·t + φ_0)` | 接收信号的瞬时相位 | rad | 线性项 + 正弦项 |
| 散射体位置 | **r_k(t)** | 第 k 个散射体的时变位置 | Scatterer time-varying position | `r_k(t) = R0 + V·t + R_rot(t)·r0` | 散射体在全局坐标系中的三维轨迹 | m | 取决于几何模型 |
| 旋转矩阵 | **R_rot(t)** | 时变旋转矩阵 | Time-varying rotation matrix | `R_rot(t) = exp(Ω̂·t)` | 将局部坐标变换到全局坐标的 3×3 旋转矩阵 | 无量纲 | exp(Ω̂·t)，Ω̂ 为 ω 的斜对称矩阵 |
| 斜对称矩阵 | **Ω̂** | 角速度斜对称矩阵 | Skew-symmetric matrix of ω | 论文 ICCT Eq.(4) | 将角速度矢量转化为叉乘运算的矩阵形式 | rad/s | 由 ω = [ω_x, ω_y, ω_z] 构造 |
| 单位方向矢量 | **n** | 雷达视线方向单位矢量 | Unit LOS direction vector | `n = (r_RX - r_TX) / ‖r_RX - r_TX‖` | 从雷达到目标的归一化方向 | 无量纲 | 近似常数 (远场) |
| 反射率函数 | **σ(x,y,z)** | 目标反射率分布 | Reflectivity function | 论文 ICCT Eq.(2) | 目标各点的散射系数分布 | 无量纲 | 由材料/形状/RCS 决定 |

### 4.3 贝塞尔级数展开

微多普勒信号可以用贝塞尔级数精确展开，这是理解谱图边带结构的数学基础：

```
s(t) = Σ_{n=-∞}^{+∞} J_n(β_m) · exp(j·2π·(f_c + f_D + n·f_rot)·t)
```

**关键性质**：
- 边带位于 f_c + f_D + n·f_rot (n 为整数)
- 第 n 个边带的幅度 ∝ |J_n(β_m)|
- β > 1 时边带数目 ≈ β + 1：例如 β=176 时约有 177 个可见边带
- 低频段（如 915 MHz）相对 28 GHz 的 β_m 小约 30 倍，MDS 更窄、更模糊；是否只有 n=0, ±1 可见还取决于叶片半径、转速、几何投影和 SNR

**汇报解释**："贝塞尔展开告诉我们：微多普勒的频谱由载波频率 f_c 处的主线和以 f_rot 为间隔、幅度由贝塞尔函数 J_n(β_m) 加权的边带组成。毫米波下 β_m 大，边带展开更宽；低频 915 MHz 下 β_m 小得多，所以 MDS 更容易模糊，需要 EMD/PCA/LSTM 这类处理来提升可分性。"

---

## 5. Sionna RT / Ray Tracing 仿真参数

### 5.1 PathSolver 参数

| 参数 | 符号 | 中文含义 | 英文含义 | 代码位置 | 物理意义 | 默认值 | 对仿真结果的影响 |
|------|------|---------|---------|---------|---------|--------|----------------|
| `max_depth` | — | 最大弹跳深度 | Maximum interaction depth | PathSolver | 射线追踪中反射/透射的最大次数 | 5 (项目); 0 (LOS only) | depth=0 仅直射路径；depth=5 包含至多 5 次反射 → 路径数指数增长 |
| `los` | — | 直射路径 | Line-of-Sight | PathSolver | 是否包含收发间直射路径 | True | 关闭后仅多径，适用于遮挡场景 |
| `specular_reflection` | — | 镜面反射 | Specular reflection | PathSolver | 是否追踪镜面反射路径 | True | 开启 → 可捕获建筑/地面反射的强多径 |
| `diffuse_reflection` | — | 漫反射 | Diffuse reflection | PathSolver | 是否追踪漫反射路径 | False (项目); True (论文 ICCT mmWave) | mmWave 下漫反射显著；开启后计算量大幅增加 |
| `diffraction` | — | 衍射 | Diffraction | PathSolver | 是否追踪衍射路径 | False | Sub-6 绕射重要；mmWave 绕射弱 |
| `refraction` | — | 折射/透射 | Refraction | PathSolver | 是否追踪透射路径 | False (项目) | 穿墙场景需要开启 |
| `synthetic_array` | — | 合成阵列 | Synthetic array mode | PathSolver | True: 通过相移合成阵列 (快)；False: 逐天线追踪 (慢，直接获取角度) | False (项目) | False 时可直接获取 paths.theta_t/phi_t 等角度信息 |
| `samples_per_src` | — | 每源采样数 | Monte Carlo samples per source | PathSolver | 每条源射线的蒙特卡洛采样数 | 1e6 (默认); 5e6 (项目 RadioMap) | 越多噪声越小；但计算时间线性增长 |
| `seed` | — | 随机种子 | Random seed | PathSolver | 蒙特卡洛采样的随机种子 | 42 | 固定种子保证可复现结果 |

### 5.2 Paths 输出 (多径信息)

| 属性 | 符号 | 中文含义 | Shape | 单位 | 物理含义 |
|------|------|---------|------|------|---------|
| `paths.a` | a_p | 通带复路径系数 | `[num_rx, num_tx, num_paths]` | 无量纲 | 包含幅度衰减和相位旋转的通带系数 |
| `paths.tau` | τ_p | 路径传播时延 | `[num_rx, num_tx, num_paths]` | s | d_p / c，路径传播时间 |
| `paths.doppler` | f_Δ,p | 路径多普勒频移 | `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]` | Hz | 由收发端运动产生的多普勒频移 |
| `paths.theta_t` | θ_t | 发射天顶角 (AoD Zenith) | `[num_rx, num_tx, num_paths]` | rad | 路径离开 TX 的天顶角 |
| `paths.phi_t` | φ_t | 发射方位角 (AoD Azimuth) | `[num_rx, num_tx, num_paths]` | rad | 路径离开 TX 的方位角 |
| `paths.theta_r` | θ_r | 接收天顶角 (AoA Zenith) | `[num_rx, num_tx, num_paths]` | rad | 路径到达 RX 的天顶角 |
| `paths.phi_r` | φ_r | 接收方位角 (AoA Azimuth) | `[num_rx, num_tx, num_paths]` | rad | 路径到达 RX 的方位角 |
| `paths.interaction_types` | — | 每跳交互类型 | `[num_rx, num_tx, num_paths, max_depth]` | 编码 | 0=NONE, 1=SPECULAR, 2=DIFFUSE, 4=REFRACTION, 8=DIFFRACTION |

### 5.3 CIR 生成参数 (`paths.cir()`)

| 参数 | 中文含义 | 默认值 | 对结果的影响 |
|------|---------|--------|-------------|
| `sampling_frequency` | CIR 时域采样率 [Hz] | 122.88e6 (项目) | 决定时延分辨率 Δτ = 1/f_s ≈ 8.14 ns；越高越精细 |
| `num_time_steps` | 时间步数 | 1 | >1 时利用多普勒频移产生时变 CIR；用于动态场景 |
| `normalize_delays` | 时延归一化 | False (项目) | True: τ 减去最短 τ；False: 保留绝对时延，可用于定位 |
| `out_type` | 输出类型 | `"numpy"` | `"numpy"`, `"tf"`, `"torch"`, `"jax"`, `"drjit"` |

**CIR 输出 shape**:
- `a_cpx`: `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps]`
- `tau`: `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]`

### 5.4 RadioMapSolver 参数

| 参数 | 中文含义 | 典型取值 | 对结果的影响 |
|------|---------|---------|-------------|
| `center` | 测量平面中心 [m] | [0,0,2] (项目) | 决定覆盖区域的位置 |
| `size` | 测量平面尺寸 [m] | [60,60] (项目) | 决定覆盖面积 |
| `cell_size` | 格点分辨率 [m] | [2,2] (3.5G); [1,1] (28G) | cell 减半 → 计算量 ×4；mmWave 需更细格点 |
| `samples_per_tx` | 每 TX 蒙特卡洛采样数 | 5,000,000 (项目) | 更多采样 → 噪声更小，计算线性增长 |

**RadioMap 输出**:
- `path_gain`: `[num_tx, cells_y, cells_x]` 线性路径增益
- `rss`: `[num_tx, cells_y, cells_x]` dBm

### 5.5 CSI 张量结构

**CIR 形式** (时域信道):
```
csi_cir = {
    coefficients:  [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps]  complex64
    delays:        [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]  float64 [s]
    doppler:       [...]  [Hz]
    theta_t, phi_t, theta_r, phi_r:  [...]  [rad]
}
```

**CFR 形式** (频域信道, OFDM):
```
H[k] = Σ_p ã_p · exp(-j·2π·f_k·τ_p)   [num_tx, num_rx, K]  complex64
```

---

## 6. STFT 时频分析参数

STFT (Short-Time Fourier Transform) 是生成微多普勒谱图 (Micro-Doppler Spectrogram, MDS) 的核心信号处理工具。

| 参数 | 符号 | 中文含义 | 英文含义 | 公式/代码位置 | 物理意义 | 典型取值 | 对 MDS 结果的影响 |
|------|------|---------|---------|-------------|---------|---------|------------------|
| 窗长 | **N_win** / `n_win` | STFT 窗长度 | Window length (samples) | STFT 参数 | 每个 FFT 帧包含的采样点数 | 2048 (项目); 256 (论文 TGRS) | **核心 trade-off**：窗越长频率分辨率 Δf = f_s/N_win 越好 → 边带更清晰；但时间分辨率越差 → 快速调制被平滑 |
| 重叠率 | `overlap_ratio` | 帧重叠率 | Overlap ratio | STFT 参数 | 相邻帧之间重叠的百分比 | 0.75 (项目, 75%); 0.9375 (论文 TGRS) | 重叠越高谱图在时间轴上越平滑连续；计算量 ∝ 1/(1-overlap) |
| 跳长 | `hop_length` | 帧跳长度 | Hop length (samples) | `= N_win × (1-overlap)` | 相邻帧起始点之间的采样间隔 | 512 (项目: 2048×0.25); 16 (论文 TGRS: 256×0.0625) | 决定时间分辨率 Δt = hop/f_s |
| 频率分辨率 | **Δf** / `delta_f` | 频率分辨率 | Frequency resolution | `= f_s / N_win` | 谱图中可分辨的最小频率间隔 | Hz | 24.4 (项目: 50000/2048); 125 (论文 TGRS: 32000/256) | 决定能否分开相邻边带 (间隔 = f_rot)；Δf < f_rot 时可以分辨 |
| 时间分辨率 | **Δt** | 时间分辨率 | Time resolution | `= hop / f_s` | 连续 STFT 帧之间的时间间隔 | s | ~10.2 ms (项目: 512/50000); ~0.5 ms (论文 TGRS: 16/32000) | 决定能否跟踪快速调制变化 |
| 窗函数类型 | — | 窗函数 | Window type | `hann`, `hamming` 等 | 用于减少频谱泄漏的加窗函数 | Hann (项目); Hamming (论文 TGRS) | Hamming 旁瓣更低；Hann 主瓣稍宽；不同窗型影响边带清晰度 |
| FFT 长度 | `N_fft` | FFT 点数 | FFT length | 通常 = N_win 或更大 (zero-padding) | 1024 (论文复现, N_win=256 时) | 大于 N_win 时做 zero-padding → 频率轴更平滑 |
| 观测时长 | **T_obs** / `obs_duration` | 观测时间 | Observation duration | `= N_samples / f_s` | 总采样时长 | s | 0.2 (项目); 1.0 (论文复现); 12.0 (论文 TGRS 每个位置) | T_obs 越大 → 可见的旋转周期越多 → 微多普勒正弦模式更清晰；但要求 UAV 在观测期间保持在波束内 |
| 频谱中心化 | `fftshift` | 零中心频率对齐 | Zero-center shift | STFT 后处理 | 将零频率移到谱图中心 | True (项目) | 使微多普勒谱以零频为中心对称显示 |
| 频率显示范围 | `freq_limit` | 显示频率范围 | Display frequency range | 截断谱图频率轴 | ±300 Hz (论文复现); ±f_dev_peak×1.5 (项目) | 仅显示感兴趣的范围，避免显示无信号的频率区域 |

### STFT 分辨率 trade-off 公式

```
Δf · Δt ≥ 1/(4π)  (Gabor 不确定性原理)
```

- 增大 N_win → Δf 变小 (频率分辨率提高) → Δt 变大 (时间分辨率降低)
- 减小 N_win → Δt 变小 (时间分辨率提高) → Δf 变大 (频率分辨率降低)
- **实际操作**：先根据 f_rot 确定需要的 Δf (应 < f_rot/2)，再反推最小 N_win = f_s/Δf

**汇报解释**："STFT 是时频分析的基本工具，其精度受 Gabor 不确定性原理约束——我们无法同时获得无限好的时间分辨率和频率分辨率。在微多普勒分析中，我们需要 Δf 足够小以分辨 f_rot 间隔的边带，同时 Δt 足够小以跟踪叶片的瞬时多普勒变化。"

---

## 7. EMD 与 IMF 相关参数

EMD (Empirical Mode Decomposition) 在论文 TGRS 的微多普勒信号处理流程中用于将微多普勒分量从杂波/直达波中分离。

| 参数 | 符号 | 中文含义 | 英文含义 | 公式位置 | 物理意义 | 典型取值 |
|------|------|---------|---------|---------|---------|---------|
| 本征模态函数 | **IMF_g(n)** | 第 g 个本征模态函数 | g-th Intrinsic Mode Function | 论文 Eq.(14) | EMD 分解出的第 g 个振荡分量 | G 个 IMF + 残差，数据自适应 |
| IMF 总数 | **G** | 本征模态函数数量 | Number of IMFs | 论文 Eq.(14) | EMD 自适应确定的 IMF 个数 | 数据依赖，信号越复杂 G 越大 |
| 残差 | **ρ(n)** | 残差分量 | Residue component | 论文 Eq.(14) | EMD 分解后的剩余趋势项 | 最后剩余分量，通常为杂波/趋势 |
| 上包络 | — | 上包络线 | Upper envelope | 论文 Eq.(15) | 通过局部极大值的三次样条插值得到 | 每个 IMF 计算时确定 |
| 下包络 | — | 下包络线 | Lower envelope | 论文 Eq.(15) | 通过局部极小值的三次样条插值得到 | 同上 |
| 筛选次数 | **n_s** | 筛选迭代次数 | Number of siftings | 复杂度公式 | 每个 IMF 提取所需的筛选迭代次数 | 数据依赖，通常 10-20 次足够 |

**EMD 在微多普勒处理中的作用**：
1. 先对频谱减法和滤波后的 `R_mc(k)` 做 IDFT，得到时域信号 `r_mc(n)`
2. 对时域 `r_mc(n)` 做 EMD 分解
3. 微多普勒分量分布在高频 IMF 中 (λ1, λ2...)
4. 杂波分量分布在低频 IMF 或残差中，选择合适 IMF 重构微多普勒信号

**汇报解释**："EMD 是一种自适应信号分解方法，不需要预设基函数，可以将复杂的接收信号分解为一系列从高频到低频的本征模态函数 (IMF)。微多普勒信号本质上是高频调制分量，因此主要分布在低阶 IMF 中。"

---

## 8. PCA 降维参数

PCA (Principal Component Analysis) 在论文 TGRS 中用于将高维 MDS 图像降维为低维特征向量，作为 LSTM 的输入。

| 参数 | 符号 | 中文含义 | 英文含义 | 公式位置 | 物理意义 | 典型取值 | 对结果的影响 |
|------|------|---------|---------|---------|---------|---------|------------|
| 主成分数 | **η** / `eta` | 保留的主成分数 | Number of principal components | 论文 Eq.(19) | 降维后保留的特征维度 | 每 100 列 MDS → 5 列 (论文 TGRS) | η 太小丢失信息；η 太大则特征冗余且 LSTM 输入维度过高 |
| 协方差矩阵 | **cov(S)** | 谱图协方差矩阵 | Covariance matrix of spectrogram | 论文 Eq.(19) | MDS 各频率 bin 之间的统计相关性矩阵 | `P × P` 矩阵 | 对角化后得到主成分 |
| 特征值 | **Λ** | 特征值矩阵 | Eigenvalue matrix | 论文 Eq.(19) | 各主成分的方差 (重要性) | 按降序排列 | 特征值越大对应的主成分越重要；累积方差比决定 η 的选择 |
| 线性映射矩阵 | **M_L** | PCA 变换矩阵 | Linear mapping matrix | 论文 Eq.(19) | 将 MDS 投影到低维空间的变换矩阵 | 由特征向量组成 | 学到的投影方向 |
| 主成分表示 | **D_L** | 降维后的 MDS 表示 | Dimension-reduced MDS | 论文 Eq.(20) | `[n_time, η]` 矩阵 | — | 紧凑的特征表示；保留了 MDS 的主要变化模式 |
| 最终特征向量 | **X** | 一维特征向量 | Final feature vector | 展开 D_L 的所有列 | 将 2D 的 D_L 拉直成 1D 向量 | 775 维 (论文 TGRS) | 作为 LSTM 的输入；由 T=25 个时间步 x 每步 31 维组成 |

**PCA 时间映射**：
- 输入: MDS 图像 (频率 bins × 时间 bins)
- 处理: 每 100 列 MDS → 5 个主成分
- 输出: D_L = [n_time, η] 降维特征矩阵
- 用途: 消除 MDS 中的冗余信息，提取最重要的变化模式

**汇报解释**："PCA 将微多普勒谱图从高维像素空间压缩到低维特征空间，在保留主要微多普勒变化模式的同时降低了 LSTM 输入维度。这既减少了计算量，又滤除了噪声的随机扰动。"

---

## 9. LSTM 分类与定位模型参数

论文 TGRS 使用 LSTM 循环神经网络进行 UAV 检测、分类和 AoA 定位。

### 9.1 网络结构参数

| 参数 | 符号 | 中文含义 | 英文含义 | 物理意义 | 典型取值 (论文 TGRS) | 对性能的影响 |
|------|------|---------|---------|---------|---------------------|------------|
| LSTM 层数 | **L** | 循环网络层数 | Number of LSTM layers | 网络深度 (堆叠的 LSTM 层数) | 3 (分类); 5 (回归) | 更深 → 更强的时序建模能力；但参数量增加，容易过拟合 |
| 每层单元数 | **H** | 隐层神经元数 | Cells per layer | 每层 LSTM 的记忆单元数量 | 1024 | 更多单元 → 更大模型容量；1024 足够捕获复杂的微多普勒时序模式 |
| 时间步数 | **T** | LSTM 展开步数 | LSTM time steps | 展开的时间序列长度 | 25 (分类); 5 (回归) | 分类需要更长的时序上下文 (25个时间步的微多普勒演变)；回归可用更短窗口 |
| 输入特征维度 | **τ** | 每步输入维度 | Input feature dimension | LSTM 每个时间步的输入向量维度 | 分类: 775/25 = 31；回归: 775/5 = 155 (论文 TGRS) | 等于 PCA 降维后的特征维度 |
| 全连接层 | `W_fc, b_fc` | 最终分类/回归层 | Fully connected layer | 将 LSTM 输出映射到类别概率 (分类) 或 AoA 值 (回归) | Softmax (分类); 线性 (回归) | 分类输出 = 类别数 (3 类); 回归输出 = 1 (AoA 方位角) |

### 9.2 LSTM 门控参数

| 参数 | 符号 | 中文含义 | 功能 |
|------|------|---------|------|
| 遗忘门权重 | **W_fg** | Forget gate weights | 控制历史信息的保留/丢弃 |
| 输入门权重 | **W_ig** | Input gate weights | 控制新信息的写入 |
| 候选状态权重 | **W_cv** | Candidate cell state weights | 生成候选记忆内容 |
| 输出门权重 | **W_og** | Output gate weights | 控制当前隐状态的输出 |
| 各门偏置 | **b_fg, b_ig, b_cv, b_og** | Gate bias vectors | 对应偏置项，训练可学习 |

### 9.3 训练参数

| 参数 | 符号 | 中文含义 | 英文含义 | 典型取值 (论文 TGRS) | 对训练的影响 |
|------|------|---------|---------|---------------------|------------|
| 批大小 | `batch_size` | 训练批量大小 | Batch size | 128 | 越大训练越快但内存消耗越大；越小梯度估计噪声越大 |
| 学习率 | `lr` | Adam 初始学习率 | Initial learning rate | 0.001, 衰减 = 0.001×0.95^((κ-1)/10) | 太大不收敛；太小收敛慢；指数衰减有助于后期精细收敛 |
| 丢弃概率 | `keep_prob` | Dropout 保留概率 | Keep probability | 0.5 | 50% 丢弃 → 强正则化，防止过拟合 |
| L2 正则化 | `l2_norm` | L2 正则化系数 | L2 regularization | 用于分类损失 | 限制权重大小，提升泛化能力 |
| L1 正则化 | `l1_norm` | L1 正则化系数 | L1 regularization | 用于回归损失 (对离群值鲁棒) | 产生稀疏权重 |
| 训练集比例 | `train_ratio` | 训练/测试划分 | Train/test split | 70% / 30% | 标准 7:3 划分 |
| Adam β₁ | — | 一阶矩指数衰减率 | First moment decay | 0.9 | 控制动量累积 |
| Adam β₂ | — | 二阶矩指数衰减率 | Second moment decay | 0.999 | 控制 RMS 缩放 |
| Adam ε | — | 数值稳定项 | Smoothing term | 1e-8 | 防止除零 |

### 9.4 汇报重点

- **LSTM 适合处理微多普勒时序数据**：微多普勒具有天然的时序结构 (周期性正弦调制)，LSTM 的门控机制能有效捕获这种时序依赖
- **分类 vs 回归使用不同层数和时间步**：检测/分类/AoA 分类使用 3 层、T=25；AoA 回归使用 5 层、T=5。论文说明这是基于实验效果选择的结构，不是理论上固定的最优结构。
- **PCA + LSTM 的联合作用**：PCA 将 MDS 图像压缩为低维特征 → LSTM 学习时序模式 → 全连接层输出结果

---

## 10. 实验数据集与评价指标参数

### 10.1 实验设置参数 (论文 TGRS)

| 参数 | 符号 | 中文含义 | 典型取值 | 对结果的影响 |
|------|------|---------|---------|------------|
| 短距离半径 | — | 短距离实验 UAV 放置半径 | 2 m, 3 m | 距离越近 SNR 越高 → MDS 越清晰 → 检测/分类/定位性能越好 |
| 长距离半径 | — | 长距离实验 UAV 放置半径 | 3 m, 4 m, 5 m | 距离越远性能越差；5 m 处某些 UAV 类型定位失败率高 |
| UAV 高度 | — | UAV 放置高度 | 44 cm, 88 cm (短距离); 44 cm, 132 cm (长距离) | 更高的 UAV 可能导致机身遮挡旋翼 → MDS 变差 |
| 方向数 (短距离) | — | 短距离方位采样点数 | 13 (0°–180°, 15°间隔) | 更多方向 → 更精细的 AoA 估计粒度 |
| 方向数 (长距离) | — | 长距离方位采样点数 | 7 (0°–180°, 30°间隔) | 距离远时减少方向数以降低实验复杂度 |
| 采样时长 / 位置 | — | 每个位置记录时长 | 12 s | 更长 → 更多样本用于训练 |
| 噪声样本数 | — | 纯噪声采集样本数 | 2,040 (来自 120 s 纯噪声数据) | 用于训练二分类检测模型 (UAV vs 噪声) |

### 10.2 数据集规模 (论文 TGRS)

| 数据集 | 实例数 | 用途 |
|--------|--------|------|
| 短距离实验 | 22,464 | 短距离下的检测/分类/定位 |
| 长距离实验 | 18,144 | 长距离下的鲁棒性验证 |
| 纯噪声 | 2,040 | 二分类检测的训练负样本 |

### 10.3 评价指标

| 指标 | 符号 | 中文含义 | 英文含义 | 公式 | 典型结果 (论文 TGRS) |
|------|------|---------|---------|------|---------------------|
| F1 分数 | **F1** | 检测 F1 分数 | Detection F1 score | `F1 = 2·P·R/(P+R)` | >0.99 (短距离, LSTM); ~0.84 (长距离, H+Q, LSTM) |
| 分类准确率 | **Accuracy** | 分类准确率 | Classification accuracy | 正确分类样本数/总样本数 | 93.9% (短距离, LSTM); 88.7% (长距离, LSTM) |
| 定位误差 | **ε** | AoA 估计绝对误差 | Absolute localization error | `ε = |θ_gt − θ_est|` | 均值 1.3° (短距离, H-all); 12.7° (长距离, Q-all) |
| 成功定位率 | — | 误差在阈值内的比例 | Successful localization ratio | `ε ≤ α` 的比例 | 98.9% (短距离, ε≤15°); 83.6% (长距离, Q-all, ε≤30°) |
| 误差标准差 | **STD** | 定位误差标准差 | Standard deviation of errors | — | 越小越稳定；短距离 H-all 的 STD 最小 |
| 误差阈值 | **α** | 成功定位的误差上界 | Error threshold | — | 15° (短距离); 30° (长距离) |
| 计算时间 | — | 在线推理时间 | Computing time | — | 在线检测 O(1); 离线训练取决于数据量 |

### 10.4 汇报重点

- **短距离 vs 长距离性能差距**：距离从 2-3 m 增加到 3-5 m 后，定位误差从 ~1.3° 退化到 ~12.7°
- **UAV 类型影响**：直升机 (H) 的定位性能远好于四旋翼 (Q) — 因为单旋翼的 MDS 更干净
- **LSTM 在几乎所有指标上优于传统机器学习方法**

---

## 11. Blender 场景建模参数

### 11.1 Blender 导出设置

| 参数 | 推荐值 | 中文含义 | 说明 |
|------|--------|---------|------|
| 导出格式 | Stanford PLY (.ply) | PLY 网格格式 | ASCII 格式，Sionna RT 要求 |
| 坐标系 | Y Forward, Z Up | 前向和上方轴 | 匹配 Sionna RT 坐标系 |
| 缩放比例 | 1.0 | 单位比例 | 1 Blender unit = 1 meter |
| 三角化 | ✅ 勾选 | Triangulate Faces | Sionna RT 仅支持三角面片 |
| 应用修改器 | ✅ 勾选 | Apply Modifiers | 确保导出的网格为最终形态 |

### 11.2 场景模型几何参数 (程序化无人机)

| 参数 | 取值 | 中文含义 | 说明 |
|------|------|---------|------|
| `box_w` | 0.30 m | 机身宽度 | 中心机身 X 方向尺寸 |
| `box_d` | 0.30 m | 机身深度 | 中心机身 Y 方向尺寸 |
| `box_h` | 0.06 m | 机身高度 | 中心机身 Z 方向尺寸 (较扁) |
| `arm_len` | 0.35 m | 旋翼臂长度 | 从机身到电机的水平距离 |
| `arm_radius` | 0.015 m | 臂圆柱半径 | 臂的粗细 |
| `motor_radius` | 0.03 m | 电机圆柱半径 | 电机/旋翼座的半径 |
| `motor_height` | 0.03 m | 电机圆柱高度 | 电机/旋翼座的高度 |
| `ground_size` | 80.0 m | 地面平面宽度 | 正方形地面平面边长 |

### 11.3 ITU 无线电材料

> Sionna RT 基于 ITU-R P.2040 标准

| 材料 ID | 适用场景 | 本项目使用 |
|---------|---------|-----------|
| `metal` | 金属结构 | 无人机机身/臂/电机 (thickness=0.02 m) |
| `concrete` | 混凝土建筑 | 地面 (thickness=0.3 m) |
| `brick` | 砖墙 | — |
| `wood` | 木质结构 | — |
| `glass` | 玻璃幕墙 | — |
| `medium_dry_ground` | 中等干燥地面 | — |
| `wet_ground` | 湿润地面 | — |

**系统会自动根据 scene.frequency 更新材料电磁属性**：3.5 GHz 和 28 GHz 下同一材料的反射系数显著不同。

---

## 12. 易混淆参数对比表

以下是项目中容易混淆的参数对，需要特别区分：

### 12.1 频率类参数

| 参数 | 符号 | 中文 | 易混淆的点 | 区分方法 |
|------|------|------|-----------|---------|
| 载波频率 | f_c | 雷达/通信系统的中心频率 | 与 f_rot 易混淆 | f_c 是电磁波频率 (GHz级)；f_rot 是机械旋转频率 (百Hz级)；f_c 出现在 β_m 公式分子中 |
| 旋转频率 | f_rot | 旋翼每秒转动圈数 | 与 f_c 发音相似 | f_rot 决定边带间隔；f_rot ≈ 100 Hz，远小于 f_c |
| 多普勒频移 | f_D | 由 UAV 平动引起的频率偏移 | 与 f_mD 易混淆 | f_D 是**恒定**的频率偏移 (水平线)；f_mD 是**时变**的正弦偏移 |
| 微多普勒频率 | f_mD(t) | 由旋翼转动引起的时变频偏 | 与 f_D 易混淆 | f_mD(t) 随时间正弦变化，范围 [-f_dev_peak, +f_dev_peak] |
| 峰值频偏 | f_dev_peak | 微多普勒最大频率偏移 | 与 f_mD 易混淆 | f_dev_peak = β_m×f_rot 是一个标量常数；f_mD(t) 是时变函数 |
| 采样率 | f_s | ADC 采样频率 | 与 f_rot 无关 | f_s 必须 ≥ 2×f_dev_peak 以满足 Nyquist |
| 子载波间隔 | Δf | OFDM 子载波频率间隔 | 与 STFT 的 Δf 同名 | OFDM 的 Δf = B/K (系统设计参数)；STFT 的 Δf = f_s/N_win (分析参数) |

### 12.2 角度类参数

| 参数 | 符号 | 中文 | 易混淆的点 | 区分方法 |
|------|------|------|-----------|---------|
| 观测角/纵横角 | θ_a / β_aspect | 雷达视线、双基地角平分线与叶片速度方向之间的投影角 | 与 θ_t (AoD) 易混淆，也与 β_m 调制指数易混淆 | 它是 UAV 姿态与雷达几何的综合角度，影响 `f_mD` 的速度投影；`β_m` 才是相位调制指数 |
| 发射天顶角 | θ_t (AoD Zenith) | 路径离开 TX 的天顶角 | 与 θ 易混淆 | θ_t 是每条多径路径的出射角度属性，由 Sionna RT 的 PathSolver 输出 |
| 接收天顶角 | θ_r (AoA Zenith) | 路径到达 RX 的天顶角 | 与 θ 易混淆 | θ_r 是每条多径路径的到达角度属性 |
| 方位角 (路径) | φ_t / φ_r | 路径出射/到达的方位角 | 与双基地角 φ 易混淆 | φ_t/φ_r 是路径角度属性；φ (双基地角) 是 TX-目标-RX 的夹角 |
| 双基地角 | φ (或 φ_b) | TX→目标→RX 的夹角 | 与路径方位角 φ_t/φ_r 易混淆 | φ 是系统几何参数；φ_t/φ_r 是每径的角度属性 |
| 欧拉角 | (ψ, θ, φ) | UAV 姿态的三维旋转角 | — | Yaw/Pitch/Roll，描述 UAV 机体的空间姿态 |
| AoA (定位) | θ_AoA | 雷达对 UAV 的到达角估计 | 与 AoA Zenith (θ_r) 易混淆 | 前者是定位输出 (标量，水平面方位角)；后者是每条路径到达天顶角 |

### 12.3 其他易混淆参数

| 参数对 | 区分方法 |
|--------|---------|
| β_m (调制指数) vs f_dev_peak (峰值频偏) | β_m 是相位调制深度 (rad)；f_dev_peak = β_m×f_rot 是频率偏移 (Hz)。不要写成 `2×f_dev_peak/f_rot` |
| N_win (STFT 窗长) vs N_fft (FFT 点数) | N_win 是实际分析窗口长度；N_fft 是 FFT 计算点数 (可以 ≥ N_win 通过 zero-padding) |
| T_obs (观测时长) vs T (LSTM 时间步) | T_obs 是物理观测时间 (秒)；T 是 LSTM 模型展开步数 (无量纲) |
| η (PCA 主成分数) vs K (OFDM 子载波数) | 都表示"维度"，但 η 是 PCA 降维后的特征维度；K 是频域子载波的数量 |

---

## 13. 关键公式汇总

### 13.1 微多普勒调制指数

```
单基地:  β_m = 4π·R·cos(θ_proj) / λ = 4π·R·f_c·cos(θ_proj) / c
双基地:  β_m ≈ 4π·R·cos(θ_proj)·cos(φ_b/2) / λ

注：TGRS Eq.(4) 直接写瞬时频率 `f_mD = 2f|v(n)|cos(φ_b/2)cos(β_aspect)/v_c`，其中 `β_aspect` 是角度；本报告的 `β_m` 是相位调制指数。
```

### 13.2 峰值频率偏移

```
f_dev_peak = β_m × f_rot
```

### 13.3 叶片尖端线速度

```
v_tip = ω_rot × R = 2π·f_rot·R
```

### 13.4 叶片闪烁频率

```
f_flash = N_blades × f_rot
```

### 13.5 平动多普勒频移

```
f_D = -(2f_c/c)·(V·n)  [单基地]
f_D = -(f_c/c)·(V·(n_TX + n_RX))  [双基地]
```

### 13.6 自由空间路径损耗

```
PL_FS = 20·log10(4πd/λ)  [dB]
```

### 13.7 RMS 时延扩展

```
τ̄  = Σ_p(P_p·τ_p) / Σ_p P_p  — 功率加权平均时延
σ_τ = sqrt( Σ_p(P_p·τ_p²)/Σ_p P_p − τ̄² )  — RMS delay spread
```

### 13.8 CFR (OFDM 频域信道)

```
H[k] = Σ_{p=1}^{P} ã_p · exp(−j·2π·f_k·τ_p),  k = 0, 1, ..., K−1
f_k = (k − K/2) · Δf
```

### 13.9 STFT 频率分辨率

```
Δf = f_s / N_win
Δt = hop / f_s = N_win·(1−overlap) / f_s
```

### 13.10 贝塞尔级数展开

```
s(t) = Σ_{n=−∞}^{+∞} J_n(β_m) · exp(j·2π·(f_c + f_D + n·f_rot)·t)
```

---

## 14. 参数速查索引

### 按字母/符号索引

| 符号 | 参数名称 | 所属章节 | 页面 |
|------|---------|---------|------|
| A_blade | 叶片散射幅度 | §2.3 | |
| A_body | 机身散射幅度 | §1.1 | |
| B | 系统带宽 | §3.1 | |
| β_m | 微多普勒相位调制指数 | §4.1 | |
| batch_size | LSTM 批大小 | §9.3 | |
| cell_size | 无线电地图格点尺寸 | §5.4 | |
| Δf | OFDM 子载波间隔 / STFT 频率分辨率 | §3.1 / §6 | |
| ε | AoA 定位误差 | §10.3 | |
| f_c | 载波频率 | §3.1 | |
| f_D | 平动多普勒频移 | §4.1 | |
| f_dev_peak | 峰值频率偏移 | §4.1 | |
| f_flash | 叶片闪烁频率 | §4.1 | |
| f_mD(t) | 瞬时微多普勒频率 | §4.1 | |
| f_rot | 旋翼旋转频率 | §2.2 | |
| f_s | 采样率 | §3.1 | |
| G | IMF 总数 | §7 | |
| H | LSTM 每层单元数 | §9.1 | |
| η | PCA 主成分数 | §8 | |
| J_n(β_m) | n 阶贝塞尔函数 | §4.3 | |
| K | OFDM 子载波数 | §3.1 | |
| L | LSTM 层数 | §9.1 | |
| λ | 波长 | §3.1 | |
| max_depth | 射线追踪最大深度 | §5.1 | |
| N_blades | 每旋翼叶片数 | §2.1 | |
| N_rotors | 旋翼数 | §2.1 | |
| N_win | STFT 窗长度 | §6 | |
| ω_rot | 旋翼角速度 | §2.2 | |
| φ (φ_b) | 双基地角 | §1.2 | |
| φ_0 | 旋翼初始相位 | §1.2 | |
| θ (aspect) | 观测角 / 纵横角 | §1.2 | |
| θ_t, φ_t | 发射天顶角/方位角 (AoD) | §5.2 | |
| θ_r, φ_r | 接收天顶角/方位角 (AoA) | §5.2 | |
| P_TX | 发射功率 | §3.2 | |
| R | 叶片半径 | §2.1 | |
| R_arm | 旋翼臂长 | §1.1 | |
| SNR | 信噪比 | §3.2 | |
| T | LSTM 时间步数 | §9.1 | |
| T_obs | 观测时长 | §6 | |
| τ_p | 路径时延 | §5.2 | |
| σ_τ | RMS 时延扩展 | §13.7 | |
| v_tip | 叶片尖端线速度 | §2.2 | |
| V_body | 机身平动速度 | §1.2 | |

---

> **文档版本**: v1.0 | **生成日期**: 2026/05/27 | **适用项目**: SionnaEM  
> **参考文献**:
> [1] "Micro-Doppler Signature Simulation of Multirotor UAVs Using Ray Tracing", ICCT 2025  
> [2] "Micro-Doppler Signature-Based Detection, Classification and Localization of Small UAV", IEEE TGRS 2021  
> [3] Sionna RT v2.0.1 Documentation
