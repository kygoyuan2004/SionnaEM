# Blender → Sionna RT 场景导入流程

## 概述

Sionna RT 使用 **Mitsuba XML** 格式描述场景，网格文件使用 **PLY** 格式。本工具提供从 Blender 导出到 Sionna RT 加载的完整流程。

## 工作流步骤

### Step 1: 在 Blender 中建模

1. 在 Blender 中创建场景物体（建筑、无人机等）
2. 确保使用 **Z-up 坐标系**（Blender 2.8+ 默认）
3. 对每个需要不同无线电材质的物体分别导出
4. **注意**：Sionna RT 单位是米，确保 Blender 中 1 unit = 1 meter

### Step 2: 导出 PLY 文件

**Blender 导出设置**（File → Export → Stanford PLY (.ply)）：

| 设置项 | 推荐值 |
|--------|--------|
| Format | **ASCII**（Sionna RT 支持 ASCII 和 Binary，ASCII 更易调试） |
| Forward Axis | **Y Forward** |
| Up Axis | **Z Up** |
| Scale | 1.0 |
| Objects → Apply Modifiers | ✅ 勾选 |
| Geometry → Triangulate Faces | ✅ 勾选（Sionna RT 需要三角面片） |
| Geometry → UV Coordinates | 可选 |
| Geometry → Color Attributes | 不勾选 |

**导出目录结构**：
```
my_scene/
├── meshes/
│   ├── building_1.ply
│   ├── building_2.ply
│   ├── ground.ply
│   └── drone_body.ply
├── materials.json          ← 材质映射（手动创建）
└── my_scene.xml            ← 由工具自动生成
```

### Step 3: 创建材质映射 JSON

创建 `materials.json`，为每个 PLY 文件指定无线电材料：

```json
{
    "building_1": {"material": "concrete", "thickness": 0.3},
    "building_2": {"material": "brick", "thickness": 0.2},
    "ground":     {"material": "medium_dry_ground", "thickness": 0.5},
    "drone_body": {"material": "metal", "thickness": 0.02}
}
```

**可用的 ITU 无线电材料**：

| 材料 | 适用场景 |
|------|---------|
| `metal` | 金属结构、无人机机身 |
| `concrete` | 混凝土建筑 |
| `brick` | 砖墙 |
| `wood` | 木质结构 |
| `glass` | 玻璃幕墙 |
| `plasterboard` | 石膏板内墙 |
| `marble` | 大理石 |
| `ceiling_board` | 天花板 |
| `fiber_glass` | 玻璃纤维 |
| `tiles` | 瓷砖 |
| `floorboard` | 地板 |
| `very_dry_ground` | 极干燥地面 |
| `medium_dry_ground` | 中等干燥地面 |
| `wet_ground` | 湿润地面 |

### Step 4: 生成场景 XML

```bash
python generate_scene_xml.py \
    --ply_dir ./meshes \
    --material_map ./materials.json \
    --output ./my_scene.xml \
    --scene_name "my_custom_scene"
```

### Step 5: 在 Sionna RT 中加载

```python
import sionna
from sionna.rt import load_scene, PlanarArray, Transmitter, Receiver, Camera

# 加载自定义场景
scene = load_scene("my_scene/my_scene.xml")
scene.frequency = 28e9  # 或 3.5e9

# 配置天线
scene.tx_array = PlanarArray(
    num_rows=1, num_cols=1,
    pattern="iso", polarization="V"
)
scene.rx_array = PlanarArray(
    num_rows=1, num_cols=1,
    pattern="iso", polarization="V"
)

# 添加基站
tx = Transmitter(
    name="bs_0",
    position=[10.0, 0.0, 10.0],
    orientation=[0, 0, 0],
    power_dbm=44
)
scene.add(tx)

# 添加接收机
rx = Receiver(name="ue", position=[0, 0, 1.5])
scene.add(rx)

# 验证：预览场景（Jupyter 中）
scene.preview()

# 验证：计算路径
from sionna.rt import PathSolver
paths = PathSolver()(scene, max_depth=5)
print(f"找到 {paths.a.shape[-1]} 条路径（第一个 TX-RX 对）")

# 验证：生成无线电地图
from sionna.rt import RadioMapSolver
rm = RadioMapSolver()(
    scene,
    cell_size=[1.0, 1.0],
    samples_per_tx=5_000_000,
)
print(f"无线电地图: {rm.path_gain.shape}")
```

## 故障排除

### 场景加载失败

1. 检查 XML 格式是否正确（尝试用浏览器打开看是否有 XML 解析错误）
2. 确保 `meshes/` 目录与 XML 在同一目录下
3. 检查 PLY 文件路径在 XML 中是否正确（相对路径）

### PLY 文件无法显示

1. 确保 PLY 是三角面片（在 Blender 中添加 Triangulate 修改器）
2. 确保 PLY 格式正确：`ply` 头 + `format ascii 1.0` + `element vertex` + `element face`
3. 尝试在 Sionna RT 的 `load_scene()` 中设置 `remove_duplicate_vertices=True`

### 场景材质不对

1. 检查 `materials.json` 中的名称是否与 PLY 文件名（不含扩展名）完全匹配
2. 未在映射中的网格会使用 `--default_material` 参数指定的材料（默认 metal）
3. 检查材料名称是否在 ITU 标准列表中（工具会输出警告）

## 简化备用方案：程序化生成场景

如果 Blender 导出流程过于复杂，可以直接在 Sionna RT 中用代码创建场景：

```python
# 创建空场景
scene = load_scene()  # None = 空场景

# 添加基本几何体（需要 Sionna RT 的场景编辑功能）
# 或者生成包含简单 box/wall 的 XML 文件
```
