# SionnaEM 完整变量参考手册

> 本文档为 Blender → Sionna RT → Pathloss + CSI 管线中所有变量的完整参考。
> 标注每个变量的名称、类型、维度（shape）、物理含义、单位和计算公式。
> 基于 Sionna RT v2.0.1。

---

## 1. 场景参数 (BlenderSceneSpec)

> 定义于 `src/step8_blender_scene_interface.py`

| 字段 | 类型 | 默认值 | 物理含义 | 单位 |
|------|------|--------|---------|------|
| `scene_xml_path` | `str \| None` | `None` | Mitsuba XML 场景文件路径（mitsuba-blender 插件直出方式） | — |
| `ply_dir` | `str \| None` | `None` | PLY 网格文件目录（Blender PLY 导出方式） | — |
| `material_map_path` | `str \| None` | `None` | 材质映射 JSON 文件路径（配合 ply_dir 使用） | — |
| `scene_name` | `str` | `"drone_scene"` | 场景名称标识 | — |
| `bs_positions` | `list[(str,[float,float,float])]` | 3 个基站三角分布 | 基站列表 (名称, [x,y,z]) | m (x,y,z) |
| `drone_position` | `[float, float, float]` | `[0,0,3]` | 无人机接收机初始位置 | m (x,y,z) |
| `map_center` | `[float, float, float]` | `[0,0,2]` | 无线电地图测量平面中心 | m (x,y,z) |
| `map_size` | `[float, float]` | `[60, 60]` | 测量平面尺寸 (width, height) | m |
| `cell_sizes` | `dict` | `{"3.5GHz":(2,2), "28GHz":(1,1)}` | 各频段格点分辨率 | m |
| `max_depth` | `int` | `5` | 射线最大弹跳次数 | — |
| `samples_per_tx` | `int` | `5,000,000` | RadioMapSolver 每发射机蒙特卡洛采样数 | — |

### 1.1 默认基站位置

| 名称 | 位置 [x,y,z] m | 说明 |
|------|----------------|------|
| `bs_0` | `[20.0, 20.0, 8.0]` | 东北角，高 8m |
| `bs_1` | `[-20.0, -20.0, 8.0]` | 西南角，高 8m |
| `bs_2` | `[20.0, -20.0, 8.0]` | 东南角，高 8m |

---

## 2. 场景物理参数 (Scene)

> 定义于 `sionna.rt.scene.Scene`

| 属性 | 类型 | 默认值 | 物理含义 | 单位 |
|------|------|--------|---------|------|
| `scene.frequency` | `mi.Float` | `3.5e9` | 载波频率 f_c，设置后自动更新波长、波数 | Hz |
| `scene.wavelength` | `mi.Float` | 导出 λ = c / f | 波长 | m |
| `scene.wavenumber` | `mi.Float` | 导出 k = 2π / λ | 波数 | rad/m |
| `scene.bandwidth` | `mi.Float` | `1e6` | 带宽 | Hz |
| `scene.temperature` | `mi.Float` | `293` | 环境温度 | K |
| `scene.thermal_noise_power` | `mi.Float` | 导出 k·T·B | 热噪声功率 | W |

### 2.1 本项目使用的两个频段

| 频段 | 频率 f_c | 波长 λ | 特点 |
|------|---------|--------|------|
| Sub-6 GHz | 3.5 GHz | 85.7 mm | 宏覆盖，绕射强，穿透性好 |
| mmWave | 28 GHz | 10.7 mm | 小基站，带宽大，微多普勒调制指数 β 大 |

**自由空间路径损耗**: `PL_FS = 20·log10(4πd / λ)` [dB]

---

## 3. 天线阵列参数 (PlanarArray)

> 定义于 `sionna.rt.antenna_array.PlanarArray`

| 参数 | 类型 | 默认值 | 物理含义 | 单位 |
|------|------|--------|---------|------|
| `num_rows` | `int` | 必填 | 天线行数 M_r | — |
| `num_cols` | `int` | 必填 | 天线列数 M_c | — |
| `vertical_spacing` | `float` | `0.5` | 垂直阵元间距 d_v / λ | 波长 λ |
| `horizontal_spacing` | `float` | `0.5` | 水平阵元间距 d_h / λ | 波长 λ |
| `pattern` | `str` | 必填 | 天线方向图: `"iso"`, `"dipole"`, `"hw_dipole"`, `"tr38901"` | — |
| `polarization` | `str` | — | 极化: `"V"`, `"H"`, `"VH"`, `"cross"` | — |
| `polarization_model` | `str` | `"tr38901_2"` | 极化模型: `"tr38901_1"` (完整) 或 `"tr38901_2"` (简化) | — |

**总天线数**: N_ant = num_rows × num_cols（双极化时 × 2）

**本项目使用**:
```python
PlanarArray(num_rows=1, num_cols=1, pattern="iso", polarization="V")
```

---

## 4. 发射机参数 (Transmitter)

> 定义于 `sionna.rt.radio_devices.transmitter.Transmitter`

| 参数 | 类型 | 默认值 | 物理含义 | 单位 |
|------|------|--------|---------|------|
| `name` | `str` | 必填 | 设备唯一标识 | — |
| `position` | `mi.Point3f` | 必填 | 发射天线相位中心位置 r_TX | m (x,y,z) |
| `orientation` | `mi.Point3f` | `[0,0,0]` | 欧拉角 (α, β, γ) | rad |
| `look_at` | `mi.Point3f` | `None` | 天线指向目标点（与 orientation 互斥） | m |
| `velocity` | `mi.Vector3f` | `[0,0,0]` | 速度矢量 v_TX | m/s |
| `power_dbm` | `mi.ScalarFloat` | `44` | 发射功率 P_TX (~25 W @ 44 dBm) | dBm |
| `color` | `(R,G,B)` | 红色 | 渲染显示颜色 | — |

**本项目使用**: `power_dbm=44`（标准基站功率）

---

## 5. 接收机参数 (Receiver)

> 定义于 `sionna.rt.radio_devices.receiver.Receiver`

| 参数 | 类型 | 默认值 | 物理含义 | 单位 |
|------|------|--------|---------|------|
| `name` | `str` | 必填 | 设备唯一标识 | — |
| `position` | `mi.Point3f` | 必填 | 接收天线相位中心位置 r_RX | m (x,y,z) |
| `orientation` | `mi.Point3f` | `[0,0,0]` | 欧拉角 (α, β, γ) | rad |
| `look_at` | `mi.Point3f` | `None` | 天线指向目标点 | m |
| `velocity` | `mi.Vector3f` | `[0,0,0]` | 速度矢量 v_RX | m/s |

**注意**: Receiver 无 `power_dbm` 参数，接收功率通过传播路径计算得出。

---

## 6. 路径求解器参数 (PathSolver)

> 定义于 `sionna.rt.path_solvers.path_solver.PathSolver`

| 参数 | 类型 | 默认值 | 物理含义 |
|------|------|--------|---------|
| `max_depth` | `int` | `3` | 最大交互次数（弹跳上限） |
| `max_num_paths_per_src` | `int` | `1e6` | 每条源射线最大路径数 |
| `samples_per_src` | `int` | `1e6` | 每条源射线蒙特卡洛采样数 |
| `synthetic_array` | `bool` | `True` | True: 相移合成阵列（快）；False: 逐天线射线追踪 |
| `los` | `bool` | `True` | 启用直射路径 (LoS) |
| `specular_reflection` | `bool` | `True` | 启用镜面反射 |
| `diffuse_reflection` | `bool` | `False` | 启用漫反射 |
| `refraction` | `bool` | `True` | 启用折射/透射 |
| `diffraction` | `bool` | `False` | 启用衍射 |
| `edge_diffraction` | `bool` | `False` | 启用自由浮动边缘衍射 |
| `seed` | `int` | `42` | 随机种子 |

**本项目使用**: `max_depth=5`, `los=True`, `specular_reflection=True`, `diffuse_reflection=False`, `diffraction=False`

---

## 7. Paths 输出 — 完整参考

> `paths = PathSolver()(scene, ...)` 返回的 Paths 对象

### 7.1 核心属性

| 属性 | Shape | 含义 | 单位 |
|------|-------|------|------|
| `paths.a` | `[num_rx, num_tx, num_paths, ...]` | 复通带路径系数 | 无量纲 |
| `paths.tau` | `[num_rx, num_tx, num_paths]` | 传播时延 | s |
| `paths.doppler` | `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]` | 路径多普勒频移 f_Δ | Hz |
| `paths.theta_t` | `[num_rx, num_tx, num_paths]` | 发射天顶角 (Departure Zenith) | rad |
| `paths.phi_t` | `[num_rx, num_tx, num_paths]` | 发射方位角 (Departure Azimuth) | rad |
| `paths.theta_r` | `[num_rx, num_tx, num_paths]` | 接收天顶角 (Arrival Zenith) | rad |
| `paths.phi_r` | `[num_rx, num_tx, num_paths]` | 接收方位角 (Arrival Azimuth) | rad |
| `paths.interaction_types` | `[num_rx, num_tx, num_paths, max_depth]` | 每跳交互类型编码 | — |

### 7.2 交互类型编码

| 值 | 含义 |
|----|------|
| 0 | NONE |
| 1 | SPECULAR (镜面反射) |
| 2 | DIFFUSE (漫反射) |
| 4 | REFRACTION (折射) |
| 8 | DIFFRACTION (衍射) |

### 7.3 多普勒频移公式

```
f_Δ = (f_c / c) · (v_TX · û_TX - v_RX · û_RX)
```

其中 `û_TX`, `û_RX` 分别为发射/接收方向的单位向量。

---

## 8. CIR 输出 (paths.cir())

> `a_cpx, tau = paths.cir(**kwargs)`

| 输出 | Shape | 类型 | 物理含义 | 单位 |
|------|-------|------|---------|------|
| `a_cpx` | `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps]` | complex64 | 复基带 CIR 系数 ã_p | 无量纲 |
| `tau` | `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]` | float64 | 路径传播时延 τ_p | s |

### 8.1 CIR 参数

| 参数 | 类型 | 默认值 | 含义 | 单位 |
|------|------|--------|------|------|
| `sampling_frequency` | `float` | `1.0` | CIR 时域采样率 f_s | Hz |
| `num_time_steps` | `int` | `1` | 时间步数（利用多普勒生成时变 CIR） | — |
| `normalize_delays` | `bool` | `True` | True: 时延相对最早到达路径归一化 | — |
| `reverse_direction` | `bool` | `False` | True: 交换收发角色（信道互易性） | — |
| `out_type` | `str` | `"drjit"` | 输出类型: `"numpy"`, `"tf"`, `"torch"`, `"jax"`, `"drjit"` | — |

### 8.2 基带转换公式

```
ã_p = a_p · exp(-j·2π·f_c·τ_p) · exp(j·2π·f_Δ,p·t)
```

其中 `a_p` 为通带路径系数，`f_c` 为载频，`f_Δ,p` 为多普勒频移。

**本项目使用**: `sampling_frequency=122.88e6`, `normalize_delays=False`, `out_type="numpy"`

---

## 9. CFR 重建 (OFDM)

> 从 CIR 重建频域信道频率响应 (Channel Frequency Response)

### 9.1 OFDM 子载波模型

```
K: 子载波数量 (--num-subcarriers, 默认 1024)
Δf: 子载波间隔 = B / K
f_k: 第 k 个子载波的频率偏移 = (k - K/2) · Δf,  k = 0,...,K-1
```

### 9.2 CFR 重建公式

```
H_tx,rx[k] = Σ_p ã_p · exp(-j·2π·f_k·τ_p)
```

其中:
- `ã_p` = `a_cpx[rx, 0, tx, 0, p, 0]` 为复基带 CIR 系数
- `τ_p` = `tau[rx, 0, tx, 0, p]` 为时延 [s]
- `f_k` 为第 k 个子载波的频率偏移 [Hz]

**输出 `H_cfr` shape**: `[num_tx, num_rx, K]` complex64

### 9.3 OFDM 参数

| 参数 | CLI 选项 | 默认值 | 含义 | 单位 |
|------|---------|--------|------|------|
| K | `--num-subcarriers` | 1024 | OFDM 子载波数 | — |
| B | `--bandwidth` | 122.88e6 | 系统带宽 | Hz |
| Δf | (导出) | B / K = 120 kHz | 子载波间隔 | Hz |

---

## 10. RadioMapSolver 输出

> `radio_map = RadioMapSolver()(scene, ...)` 返回 PlanarRadioMap 对象

### 10.1 RadioMapSolver 参数

| 参数 | 类型 | 默认值 | 含义 | 单位 |
|------|------|--------|------|------|
| `center` | `mi.Point3f` | 场景中心 z=1.5m | 测量平面中心 | m |
| `orientation` | `mi.Point3f` | 平行 XY 平面 | 测量平面朝向 | rad |
| `size` | `mi.Point2f` | 覆盖整个场景 | 测量平面尺寸 (width, height) | m |
| `cell_size` | `mi.Point2f` | `(10, 10)` | 格点分辨率 | m |
| `samples_per_tx` | `int` | `1e6` | 每发射机采样数 | — |
| `precoding_vec` | `(real, imag)` | 等幅同相 | 预编码向量 | — |

### 10.2 RadioMap 输出属性

| 属性 | Shape | 含义 | 单位 |
|------|-------|------|------|
| `path_gain` | `[num_tx, cells_y, cells_x]` | 每格点路径增益 G（线性值） | 无量纲 |
| `rss` | `[num_tx, cells_y, cells_x]` | 接收信号强度 (G × P_TX) | dBm |
| `sinr` | `[num_tx, cells_y, cells_x]` | 信干噪比 | dB |

**路径增益转 dB**: `G_dB = 10 · log10(G + ε)`

**关键结论**: `path_gain[0]` = BS0 的信道图, `path_gain[1]` = BS1 的信道图 — 每个基站可独立拆分。

### 10.3 性能建议

| 频段 | 建议 cell_size | 原因 |
|------|---------------|------|
| 3.5 GHz | [2.0, 2.0] m | 波长 ~8.6 cm, 低分辨率即可 |
| 28 GHz | [0.5~1.0, 0.5~1.0] m | 波长 ~1 cm, 需要更高分辨率 |

- cell_size 减半 → 计算量 × 4
- samples_per_tx 翻倍 → 计算时间线性增长，噪声减少

---

## 11. 导出信道指标 (Derived Metrics)

### 11.1 路径损耗 (Pathloss)

| 指标 | 公式 | 单位 |
|------|------|------|
| Per-path pathloss | `PL_p = -20·log10(|ã_p| + ε)` | dB |
| Combined pathloss | `PL_combined = -10·log10(Σ_p |ã_p|² + ε)` | dB |
| Free-space pathloss (ref) | `PL_FS = 20·log10(4πd_min / λ)` | dB |

其中 `d_min` 为最短路径的传播距离 = `c · min(τ_p)`.

### 11.2 均方根时延扩展 (RMS Delay Spread)

```
功率加权平均时延:  τ̄ = Σ_p (|ã_p|² · τ_p) / Σ_p |ã_p|²
均方时延:          τ̄² = Σ_p (|ã_p|² · τ_p²) / Σ_p |ã_p|²
RMS Delay Spread:   σ_τ = sqrt(Σ_p(P_p·τ_p²)/Σ_p P_p - τ̄²)
```

**物理含义**: 描述多径信道的时间色散程度。σ_τ 越大，频率选择性衰落越严重。

**典型值范围**: 室内 ~10-50 ns, 室外微蜂窝 ~100-1000 ns.

### 11.3 角度扩展 (Angular Spread)

```
功率加权平均角度:  θ̄ = Σ_p (|ã_p|² · θ_p) / Σ_p |ã_p|²
角度扩展:          σ_θ = sqrt(Σ_p (|ã_p|² · (θ_p - θ̄)²) / Σ_p |ã_p|²)
```

分别计算 AoD (θ_t, φ_t) 和 AoA (θ_r, φ_r) 四个方向的角度扩展。

### 11.4 CSI 条件数 (Condition Number)

```
H 矩阵的 SVD: H = U · Σ · V^H
条件数: κ = σ_max / σ_min
```

其中 σ_max, σ_min 分别为最大和最小奇异值。κ 越大，信道越不均衡。

---

## 12. ITU 无线电材料

> Sionna RT 基于 ITU-R P.2040 标准

| 材料 ID | 适用场景 | 相对介电常数范围 | 电导率范围 |
|---------|---------|-----------------|-----------|
| `metal` | 金属结构、无人机机身 | — (PEC) | — |
| `concrete` | 混凝土建筑 | 5.31 ~ 5.06 | 0.03 ~ 0.24 |
| `brick` | 砖墙 | 3.75 ~ 3.66 | 0.038 ~ 0.04 |
| `wood` | 木质结构 | 1.99 | 0.004 ~ 0.02 |
| `glass` | 玻璃幕墙 | 6.27 ~ 6.05 | 0.004 ~ 0.09 |
| `plasterboard` | 石膏板内墙 | 2.94 | 0.010 ~ 0.028 |
| `marble` | 大理石 | 7.0 | 0.016 ~ 0.037 |
| `ceiling_board` | 天花板 | 1.50 | 0.001 ~ 0.010 |
| `fiber_glass` | 玻璃纤维 | 1.0 | 0.002 ~ 0.008 |
| `tiles` | 瓷砖 | 7.0 | 0.040 ~ 0.096 |
| `floorboard` | 地板 | 3.66 | 0.037 ~ 0.065 |
| `very_dry_ground` | 极干燥地面 | 3.0 | 0.00015 ~ 0.0004 |
| `medium_dry_ground` | 中等干燥地面 | 15.0 | 0.035 ~ 0.087 |
| `wet_ground` | 湿润地面 | 30.0 | 0.15 ~ 0.37 |

**注意**: 设置 `scene.frequency` 会触发所有材料的 `frequency_update()`，自动更新电磁属性。3.5 GHz 和 28 GHz 下的反射/透射系数显著不同。

**厚度参数 `thickness`**: 可配置，默认 0.1 m。

**本项目程序化场景使用**:
- 无人机 (body + arm + motor): `metal`, thickness=0.02 m
- 地面: `concrete`, thickness=0.3 m

---

## 13. CSI 张量结构总览

### 13.1 CIR 形式 (`csi_cir`)

```
{
    "coefficients":  a_cpx,      # complex64, [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths, num_time_steps]
    "delays":        tau,        # float64,   [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]  [s]
    "doppler":       doppler,    # [num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]  [Hz]
    "theta_t":       θ_t,        # [num_rx, num_tx, num_paths]  [rad] — AoD zenith
    "phi_t":         φ_t,        # [num_rx, num_tx, num_paths]  [rad] — AoD azimuth
    "theta_r":       θ_r,        # [num_rx, num_tx, num_paths]  [rad] — AoA zenith
    "phi_r":         φ_r,        # [num_rx, num_tx, num_paths]  [rad] — AoA azimuth
    "num_paths":     int,
    "num_tx":        int,
    "sampling_frequency": float  # [Hz]
}
```

### 13.2 CFR 形式 (`csi_cfr`, 仅当 --ofdm)

```
{
    "H":                  H_cfr,       # complex64, [num_tx, num_rx, K]
    "K":                  int,         # 子载波数
    "subcarrier_spacing": float,       # Δf [Hz]
    "bandwidth":          float,       # B [Hz]
    "f_offsets":          f_k,         # float64, [K] — 子载波频率偏移 [Hz]
}
```

### 13.3 .npz 数据文件

保存路径: `pipeline/data/pipeline_csi_{freq}.npz`

包含 keys: `a_cpx`, `tau`, `path_gain`, `path_gain_db`, `rss`, `freq_hz`, 以及各导出指标。

---

## 14. 单位速查表

| 物理量 | 符号 | 标准单位 | 其他常用单位 |
|--------|------|---------|-------------|
| 频率 | f_c, f | Hz | GHz (= 1e9 Hz), MHz (= 1e6 Hz) |
| 波长 | λ | m | mm (= 1e-3 m) |
| 时延 | τ | s | ns (= 1e-9 s) |
| 距离 | d, r | m | km (= 1e3 m) |
| 功率 | P | W | dBm (= 10·log10(P / 1mW)) |
| 路径增益 | G | 无量纲 | dB (= 10·log10(G)) |
| 多普勒频移 | f_Δ | Hz | kHz (= 1e3 Hz) |
| 角度 | θ, φ | rad | deg (= rad · 180 / π) |
| 速度 | v | m/s | km/h (= m/s / 3.6) |
| 温度 | T | K | °C (= K - 273.15) |
| 带宽 | B | Hz | MHz (= 1e6 Hz) |

---

## 15. 物理常数

| 常数 | 符号 | 值 | 单位 |
|------|------|----|------|
| 光速 | c | 2.99792458 × 10^8 | m/s |
| 玻尔兹曼常数 | k | 1.380649 × 10^-23 | J/K |
