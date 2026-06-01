# Sionna RT 基站端可配置参数参考手册

> 本文档整理 Sionna RT v2.0.1 中基站（发射机）端所有可配置参数，按功能类别组织。每项参数标注类型、默认值、含义，以及项目当前使用值。

---

## 1. 天线阵列参数 (`PlanarArray`)

**定义位置**: `sionna.rt.antenna_array.PlanarArray`

```python
PlanarArray(*,
    num_rows: int,
    num_cols: int,
    vertical_spacing: float = 0.5,
    horizontal_spacing: float = 0.5,
    pattern: str,
    **kwargs)
```

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `num_rows` | `int` | 必填 | 天线阵列行数 |
| `num_cols` | `int` | 必填 | 天线阵列列数 |
| `vertical_spacing` | `float` | `0.5` | 垂直阵元间距 [波长 λ] |
| `horizontal_spacing` | `float` | `0.5` | 水平阵元间距 [波长 λ] |
| `pattern` | `str` | 必填 | 天线方向图名称，见下表 |
| `polarization` | `str` | — | 极化方式，见下表（通过 kwargs 传入） |
| `polarization_model` | `str` | `"tr38901_2"` | 极化模型（通过 kwargs 传入） |

**总天线数**: `num_rows × num_cols`（每极化方向单独计数，双极化时翻倍）

### 1.1 可用天线方向图 (`pattern`)

| 方向图 | 说明 |
|--------|------|
| `"iso"` | 全向天线（各方向增益为 1） |
| `"dipole"` | 半波偶极子 |
| `"hw_dipole"` | 半波偶极子（变体） |
| `"tr38901"` | 3GPP TR 38.901 标准天线方向图 |

### 1.2 可用极化方式 (`polarization`)

| 极化 | 极化倾斜角 | 说明 |
|------|----------|------|
| `"V"` | `[0.0]` | 垂直极化，单极化 |
| `"H"` | `[π/2]` | 水平极化，单极化 |
| `"VH"` | `[0.0, π/2]` | V+H 双极化 |
| `"cross"` | `[-π/4, π/4]` | ±45° 交叉极化，双极化 |

### 1.3 可用极化模型 (`polarization_model`)

| 模型 | 说明 |
|------|------|
| `"tr38901_1"` | 3GPP TR 38.901 Model-1：完整角度相关极化旋转 |
| `"tr38901_2"` | 3GPP TR 38.901 Model-2：简单极化倾斜旋转（**默认**） |

### 1.4 项目当前使用值

```python
scene.tx_array = PlanarArray(
    num_rows=1, num_cols=1,
    vertical_spacing=0.5, horizontal_spacing=0.5,
    pattern="iso", polarization="V",
)
```

**注**: 大尺度无线电地图库 (`sionna-large-radio-maps`) 使用 `num_rows=2, num_cols=16, pattern="tr38901", polarization="V"`（32 单元阵列）。

---

## 2. 载频、带宽与环境参数 (`Scene`)

**定义位置**: `sionna.rt.scene.Scene`

| 属性 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `scene.frequency` | `mi.Float` | `3.5e9` Hz | 载波频率 [Hz]，设置后自动更新波长、波数、材料属性 |
| `scene.bandwidth` | `mi.Float` | `1e6` Hz | 带宽 [Hz]，用于热噪声计算 |
| `scene.temperature` | `mi.Float` | `293` K | 环境温度 [K] |
| `scene.wavelength` | `mi.Float` | 导出 | 波长 λ = c / f |
| `scene.wavenumber` | `mi.Float` | 导出 | 波数 k = 2π / λ |
| `scene.angular_frequency` | `mi.Float` | 导出 | 角频率 ω = 2πf |
| `scene.thermal_noise_power` | `mi.Float` | 导出 | 热噪声功率 k·T·B |

**重要**: 设置 `scene.frequency` 会触发所有无线电材料的 `frequency_update()`，自动更新材料在不同频率下的电磁属性。3.5 GHz 和 28 GHz 下同一材料的反射/透射系数会显著不同。

### 项目使用的两个目标频段

| 频段 | 频率 | 波长 | 适用场景 |
|------|------|------|---------|
| Sub-6 GHz | 3.5 GHz | 85.7 mm | 宏覆盖，穿透性好 |
| mmWave | 28 GHz | 10.7 mm | 小基站，微多普勒调制指数大 |

---

## 3. 发射机参数 (`Transmitter`)

**定义位置**: `sionna.rt.radio_devices.transmitter.Transmitter`

```python
Transmitter(name: str,
            position: mi.Point3f,
            orientation: mi.Point3f | None = None,
            look_at: mi.Point3f | Self | None = None,
            velocity: mi.Vector3f | None = None,
            power_dbm = DEFAULT_TRANSMIT_POWER_DBM,
            color: Tuple[float, float, float] = DEFAULT_TRANSMITTER_COLOR,
            display_radius: float | None = None)
```

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `name` | `str` | 必填 | 设备唯一标识名 |
| `position` | `mi.Point3f` | 必填 | 设备位置 (x, y, z) [m] |
| `orientation` | `mi.Point3f` | `[0,0,0]` | 欧拉角 (α, β, γ) [rad]，旋转顺序见 Sionna 文档 |
| `look_at` | `mi.Point3f` | `None` | 指向目标点或设备，与 orientation 二选一 |
| `velocity` | `mi.Vector3f` | `[0,0,0]` | 速度矢量 [m/s]，用于多普勒频移计算 |
| `power_dbm` | `mi.ScalarFloat` | **44 dBm** | 发射功率（~25 W） |
| `color` | `(R,G,B)` | 红色 | 在 preview/render 中的显示颜色 |
| `display_radius` | `float` | `None` | 显示半径 [m] |

**可读写属性**: `position`、`orientation`、`velocity`、`power_dbm` 均可在运行时修改。

### 项目当前使用值

```python
power_dbm = 0.0  # 微多普勒脚本中使用（低功率避免接收机饱和）
# 大规模无线电地图中使用默认值 44 dBm
```

---

## 4. 接收机参数 (`Receiver`)

**定义位置**: `sionna.rt.radio_devices.receiver.Receiver`

```python
Receiver(name: str,
         position: mi.Point3f,
         orientation: mi.Point3f | None = None,
         look_at: mi.Point3f | Self | None = None,
         velocity: mi.Vector3f | None = None,
         color: Tuple[float, float, float] = DEFAULT_RECEIVER_COLOR,
         display_radius: float | None = None)
```

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `name` | `str` | 必填 | 设备唯一标识名 |
| `position` | `mi.Point3f` | 必填 | 设备位置 (x, y, z) [m] |
| `orientation` | `mi.Point3f` | `[0,0,0]` | 欧拉角 (α, β, γ) |
| `look_at` | `mi.Point3f` | `None` | 指向目标点 |
| `velocity` | `mi.Vector3f` | `[0,0,0]` | 速度矢量 [m/s] |
| `color` | `(R,G,B)` | 绿色 | 显示颜色 |
| `display_radius` | `float` | `None` | 显示半径 |

**注意**: Receiver **没有** `power_dbm` 参数。接收信号功率通过传播路径计算得出。

---

## 5. 路径求解器参数 (`PathSolver`)

**定义位置**: `sionna.rt.path_solvers.path_solver.PathSolver`

```python
solver = PathSolver()
paths = solver(scene,
    max_depth: int = 3,
    max_num_paths_per_src: int = 1_000_000,
    samples_per_src: int = 1_000_000,
    synthetic_array: bool = True,
    los: bool = True,
    specular_reflection: bool = True,
    diffuse_reflection: bool = False,
    refraction: bool = True,
    diffraction: bool = False,
    edge_diffraction: bool = False,
    diffraction_lit_region: bool = True,
    seed: int = 42)
```

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `scene` | `Scene` | 必填 | 场景对象 |
| `max_depth` | `int` | `3` | 最大交互次数（反射/绕射等弹跳次数上限） |
| `max_num_paths_per_src` | `int` | `1e6` | 每条源射线最大路径数 |
| `samples_per_src` | `int` | `1e6` | 每条源射线的采样数（蒙特卡洛） |
| `synthetic_array` | `bool` | `True` | True: 通过相移合成阵列（快）；False: 逐天线追踪 |
| `los` | `bool` | `True` | 启用直射路径 (Line-of-Sight) |
| `specular_reflection` | `bool` | `True` | 启用镜面反射 |
| `diffuse_reflection` | `bool` | `False` | 启用漫反射 |
| `refraction` | `bool` | `True` | 启用折射/透射 |
| `diffraction` | `bool` | `False` | 启用衍射 |
| `edge_diffraction` | `bool` | `False` | 启用自由浮动边缘的衍射 |
| `diffraction_lit_region` | `bool` | `True` | 启用光照区域的衍射 |
| `seed` | `int` | `42` | 随机种子 |

**额外属性**: `PathSolver.loop_mode` — `"symbolic"`（默认，快）或 `"evaluated"`（支持自动微分）。

### 项目当前使用值

| 参数 | 值 |
|------|----|
| `max_depth` | `5` |
| `los` | `True` |
| `specular_reflection` | `True` |
| `diffuse_reflection` | `True` |
| `diffraction` | `False` |
| `edge_diffraction` | `False` |
| `refraction` | `False` |
| `synthetic_array` | `False` |

---

## 6. CIR 生成参数 (`paths.cir()`)

**定义位置**: `sionna.rt.path_solvers.paths.Paths.cir`

```python
a, tau = paths.cir(*,
    sampling_frequency: float = 1.,
    num_time_steps: int = 1,
    normalize_delays: bool = True,
    reverse_direction: bool = False,
    out_type: Literal["drjit", "jax", "numpy", "tf", "torch"] = "drjit")
```

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `sampling_frequency` | `float` | `1.0` | CIR 时域采样频率 [Hz] |
| `num_time_steps` | `int` | `1` | 时间步数（利用多普勒频移生成时变 CIR） |
| `normalize_delays` | `bool` | `True` | True: 时延相对于最早到达路径归一化；False: 保留绝对时延 |
| `reverse_direction` | `bool` | `False` | True: 交换收发角色（信道互易性） |
| `out_type` | `str` | `"drjit"` | 输出类型：`"numpy"`, `"tf"`, `"torch"`, `"jax"`, `"drjit"` |

**CIR 输出维度**:
- `a`: `[num_rx, num_time_steps, num_tx, num_tx_ants, num_paths, num_rx_ants]` — 复基带系数
- `tau`: `[num_rx, num_time_steps, num_tx, num_tx_ants, num_paths]` — 路径时延 [s]

**基带转换公式**: a_i^b = a_i × exp(-j·2πf·τ_i) × exp(j·2π·f_Δ,i·t)

其中 a_i 是通带路径系数，f 是载频，f_Δ,i 是多普勒频移。

### 项目当前使用值

```python
sampling_frequency = 122.88e6  # Hz
normalize_delays = False
out_type = "numpy"
```

---

## 7. 无线电地图求解器参数 (`RadioMapSolver`)

**定义位置**: `sionna.rt.radio_map_solvers.radio_map_solver.RadioMapSolver`

```python
rm = RadioMapSolver()(scene,
    center: mi.Point3f | None = None,
    orientation: mi.Point3f | None = None,
    size: mi.Point2f | None = None,
    cell_size: mi.Point2f = mi.Point2f(10, 10),
    measurement_surface: mi.Shape | SceneObject | None = None,
    precoding_vec: Tuple[mi.TensorXf, mi.TensorXf] | None = None,
    samples_per_tx: int = 1_000_000,
    max_depth: int = 3,
    los: bool = True,
    specular_reflection: bool = True,
    diffuse_reflection: bool = False,
    refraction: bool = True,
    diffraction: bool = False,
    edge_diffraction: bool = False,
    diffraction_lit_region: bool = True,
    seed: int = 42,
    rr_depth: int = -1,
    rr_prob: float = 0.95,
    stop_threshold: float | None = None,
    modified_scene: mi.Scene | None = None)
```

### 7.1 测量平面定义

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `center` | `mi.Point3f` | `None`（场景中心, z=1.5m） | 测量平面中心 (x, y, z) [m] |
| `orientation` | `mi.Point3f` | `None`（平行 XY 平面） | 测量平面欧拉角 (α, β, γ) [rad] |
| `size` | `mi.Point2f` | `None`（覆盖整个场景） | 测量平面尺寸 (width, height) [m] |
| `cell_size` | `mi.Point2f` | `(10, 10)` | 每个格点的尺寸 [m]，越小分辨率越高但计算量越大 |
| `measurement_surface` | `Shape/SceneObject` | `None` | 自定义测量表面网格，设置后忽略以上四个参数 |

### 7.2 波束赋形

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `precoding_vec` | `(real, imag)` | `None`（等幅同相） | 复数预编码向量，(实部张量, 虚部张量)。默认为 `[1,...,1]/sqrt(num_tx_ant)` |

### 7.3 传播参数

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `samples_per_tx` | `int` | `1e6` | 每个发射机的蒙特卡洛采样数 |
| `max_depth` | `int` | `3` | 最大弹跳次数 |
| `los` | `bool` | `True` | 直射路径 |
| `specular_reflection` | `bool` | `True` | 镜面反射 |
| `diffuse_reflection` | `bool` | `False` | 漫反射 |
| `refraction` | `bool` | `True` | 折射 |
| `diffraction` | `bool` | `False` | 衍射 |
| `edge_diffraction` | `bool` | `False` | 边缘衍射 |
| `diffraction_lit_region` | `bool` | `True` | 光照区域衍射 |
| `seed` | `int` | `42` | 随机种子 |

### 7.4 俄罗斯轮盘赌路径终止

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `rr_depth` | `int` | `-1` | 开始俄罗斯轮盘赌的深度。-1 表示禁用 |
| `rr_prob` | `float` | `0.95` | 路径继续的最大概率 |
| `stop_threshold` | `float` | `None` | 路径增益阈值 [dB]，低于此值终止路径 |

### 7.5 高级参数

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `modified_scene` | `mi.Scene` | `None` | 预扩展的 Mitsuba 场景对象，用于场景修改后计算 |

### 7.6 RadioMap 输出

`RadioMapSolver` 返回 `RadioMap` 对象（`PlanarRadioMap` 或 `MeshRadioMap`）：

**PlanarRadioMap 关键属性**:
- `path_gain`: shape `[num_tx, cells_y, cells_x]` — 每格点路径增益（线性值）
- `rss`: 接收信号强度 [dBm]（路径增益 × 发射功率）
- `sinr`: 信干噪比 [dB]（考虑多发射机干扰和热噪声）
- `center`, `orientation`, `size`, `cell_size`: 测量平面参数

**关键结论**: `path_gain[0]` 是 BS1 的图，`path_gain[1]` 是 BS2 的图 — **每个基站的信道图可以独立拆分**。

### 7.7 性能提示

- `cell_size` 减小 4 倍 → 计算量增加 16 倍
- 28 GHz 建议 `cell_size=[0.5, 0.5]`（波长 ~1 cm）
- 3.5 GHz 可以用 `cell_size=[2.0, 2.0]`（波长 ~8.6 cm）
- `samples_per_tx` 越大 → 噪声越小，但计算时间线性增长

---

## 8. 大尺度无线电地图参数

**定义位置**: `sionna-large-radio-maps/sionna_lrm/constants.py`

### 8.1 默认配置

| 参数 | 值 | 含义 |
|------|----|------|
| `DEFAULT_ANTENNA_ARRAY_PARAMS` | `{num_rows: 2, num_cols: 16, pattern: "tr38901", polarization: "V"}` | 默认 BS 天线阵列 |
| `DEFAULT_TRANSMIT_POWER_DBM` | `44` | 默认发射功率 |
| `DEFAULT_MIN_CELL_SIZE` | `5` m | 最小瓦片格点尺寸 |
| `DEFAULT_MAX_CELL_SIZE` | `100` m | 最大瓦片格点尺寸 |
| `MIN_SAMPLES_PER_TX` | `20,000,000` | 每发射机最少采样数 |
| `ASSUMED_SCENE_SIZE_MIB` | `3 × 1024` MiB | 假设场景内存占用量 |

### 8.2 基站放置参数

| 参数 | 值 | 含义 |
|------|----|------|
| `TX_Z_OFFSET_BUILDING` | `2.0` m | 建筑物上方 BS 高度偏移 |
| `TX_Z_OFFSET_NON_BUILDING` | `25.0` m | 非建筑物上方 BS 高度偏移 |
| `TX_SEARCH_DISTANCE_M` | `2000` m | 瓦片边界外额外搜索 BS 距离 |
| `TX_SEARCH_RADIUS_FACTOR` | `1.5` | BS 搜索半径乘数 |

### 8.3 测量配置

| 参数 | 值 | 含义 |
|------|----|------|
| `DEFAULT_MEASUREMENT_MESH_NAME` | `"ground"` | 默认测量表面名称 |
| 测量高度偏移 | `1.5` m | 接收机高度（地面以上） |

---

## 9. 视觉化/渲染参数

### 9.1 交互式预览 (`scene.preview()`)

```python
scene.preview(*,
    background: str = "#ffffff",
    clip_at: float | None = None,
    clip_plane_orientation: tuple = (0,0,-1),
    fov: float = 45,
    paths: Paths | None = None,
    radio_map: RadioMap | None = None,
    resolution: tuple = (655, 500),
    rm_db_scale: bool = True,
    rm_metric: str = "path_gain",
    rm_tx: int | str | None = None,
    rm_vmax: float | None = None,
    rm_vmin: float | None = None,
    rm_cmap: str | callable | None = None,
    show_devices: bool = True,
    show_orientations: bool = False,
    point_picker: bool = True)
```

| 参数 | 含义 |
|------|------|
| `background` | 背景色 (hex) |
| `clip_at` | 裁剪面偏移 [m]，用于剖切显示建筑内部 |
| `fov` | 视场角 [deg] |
| `paths` | 叠显传播路径 |
| `radio_map` | 叠显无线电地图 |
| `rm_metric` | 显示的无线电地图指标：`"path_gain"` / `"rss"` / `"sinr"` |
| `rm_tx` | 按发射机名或索引筛选显示：`None` = 全部最大值合成, `0` = BS1, `"bs_0"` = 按名称 |
| `rm_db_scale` | True: 10·log10 显示；False: 线性显示 |
| `rm_vmin/vmax` | 色标范围 [dB] |
| `show_devices` | 显示收发设备标记 |
| `show_orientations` | 显示设备朝向箭头 |
| `point_picker` | Alt+点击获取坐标 |

### 9.2 静态渲染 (`scene.render()`)

```python
scene.render(*,
    camera: Camera | str,
    num_samples: int = 128,
    resolution: tuple = (655, 500),
    return_bitmap: bool = False,
    envmap: str | None = None,
    lighting_scale: float = 1.0,
    ...)  # 其余参数同 preview()
```

| 新增参数 | 含义 |
|------|------|
| `camera` | `Camera` 实例或 `"preview"`（使用交互预览的当前视角） |
| `num_samples` | 每像素光线采样数（越大越细腻） |
| `return_bitmap` | True: 直接返回 `mi.Bitmap`；False: 返回 `plt.Figure` |
| `envmap` | 环境贴图文件路径 (.exr) 用于场景光照 |
| `rm_show_color_bar` | 显示色标条 |

### 9.3 保存到文件 (`scene.render_to_file()`)

```python
scene.render_to_file(*,
    camera: Camera | str,
    filename: str,
    ...)  # 其余参数同 render()
```

---

## 10. 场景加载参数 (`load_scene()`)

```python
scene = load_scene(
    filename: str | None = None,
    merge_shapes: bool = True,
    merge_shapes_exclude_regex: str | None = None,
    remove_duplicate_vertices: bool = False)
```

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `filename` | `str` / `None` | `None` | Mitsuba XML 场景文件路径；`None` 创建空场景 |
| `merge_shapes` | `bool` | `True` | 合并共享相同无线电材料的形状（提升 RT 性能） |
| `merge_shapes_exclude_regex` | `str` | `None` | 排除合并的形状正则表达式 |
| `remove_duplicate_vertices` | `bool` | `False` | 删除重复顶点 |

### 内置场景

| 场景 | 说明 |
|------|------|
| `sionna.rt.scene.etoile` | 巴黎凯旋门区域 |
| `sionna.rt.scene.munich` | 慕尼黑圣母教堂区域 |
| `sionna.rt.scene.florence` | 佛罗伦萨大教堂区域 |
| `sionna.rt.scene.san_francisco` | 旧金山城区 |
| `sionna.rt.scene.floor_wall` | 地面 + 垂直墙面（简单场景） |
| `sionna.rt.scene.box` 系列 | 金属/玻璃盒场景（测试用） |
| `sionna.rt.scene.simple_street_canyon` | 街道峡谷 |
| `sionna.rt.scene.simple_street_canyon_with_cars` | 街道峡谷 + 车辆 |

---

## 11. 无线电材料参数

Sionna RT 支持 ITU-R P.2040 标准定义的以下无线电材料：

| 材料 | 适用场景 |
|------|---------|
| `"metal"` | 金属结构 |
| `"concrete"` | 混凝土建筑 |
| `"brick"` | 砖墙 |
| `"wood"` | 木质结构 |
| `"glass"` | 玻璃幕墙 |
| `"plasterboard"` | 石膏板 |
| `"marble"` | 大理石 |
| `"ceiling_board"` | 天花板 |
| `"fiber_glass"` | 玻璃纤维 |
| `"tiles"` | 瓷砖 |
| `"floorboard"` | 地板 |
| `"very_dry_ground"` | 极干燥地面 |
| `"medium_dry_ground"` | 中等干燥地面 |
| `"wet_ground"` | 湿润地面 |

每种材料可配置 `thickness` 参数（默认 0.1 m）。材料属性随频率自动更新。

---

## 12. Paths 对象可用的输出属性

| 属性 | Shape | 含义 |
|------|-------|------|
| `paths.a` | `[num_rx, num_tx, num_paths, ...]` | 复通带路径系数 |
| `paths.tau` | `[num_rx, num_tx, num_paths]` | 路径传播时延 [s] |
| `paths.doppler` | `[num_rx, num_rx_ant, num_tx, num_tx_ant, num_paths]` | 路径多普勒频移 [Hz] |
| `paths.theta_t` | `[num_rx, num_tx, num_paths]` | 发射天顶角 [rad] |
| `paths.phi_t` | `[num_rx, num_tx, num_paths]` | 发射方位角 [rad] |
| `paths.theta_r` | `[num_rx, num_tx, num_paths]` | 接收天顶角 [rad] |
| `paths.phi_r` | `[num_rx, num_tx, num_paths]` | 接收方位角 [rad] |
| `paths.interaction_types` | `[num_rx, num_tx, num_paths, max_depth]` | 每跳交互类型（NONE=0, SPECULAR=1, DIFFUSE=2, REFRACTION=4, DIFFRACTION=8） |

---

## 13. 参数速查表

| 类别 | 参数 | 3.5 GHz 建议值 | 28 GHz 建议值 |
|------|------|---------------|--------------|
| 载频 | `scene.frequency` | `3.5e9` | `28e9` |
| 阵列 | `PlanarArray` | 1×1 ~ 2×16 | 1×1 ~ 2×16 |
| 方向图 | `pattern` | `"tr38901"` | `"iso"` 或 `"tr38901"` |
| 极化 | `polarization` | `"V"` | `"V"` |
| TX 功率 | `power_dbm` | 44 dBm | 30~44 dBm |
| 路径深度 | `max_depth` | 5 | 3~5 |
| 漫反射 | `diffuse_reflection` | True | True（mmWave 漫反射显著） |
| 衍射 | `diffraction` | True（Sub-6 绕射重要） | False |
| CIR 采样 | `sampling_frequency` | 122.88 MHz | 122.88 MHz |
| RM 格点 | `cell_size` | [2.0, 2.0] m | [0.5, 0.5] m |
| RM 采样 | `samples_per_tx` | 10M | 10M~50M |
