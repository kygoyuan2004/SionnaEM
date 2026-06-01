#!/usr/bin/env python3
"""
Step 8: Blender 场景接口 — Sionna RT 场景加载统一入口

支持三种场景来源（按优先级）：
  1. Mitsuba XML 直出（推荐）：Blender + mitsuba-blender 插件 → 直接导出 .xml → load_scene()
  2. PLY 中转（现有方式）：Blender → 导出 PLY → generate_scene_xml.py → load_scene()
  3. 程序化回退（Blender 未安装）：代码直接生成 PLY + XML

用法:
    from step8_blender_scene_interface import BlenderSceneSpec, load_scene_from_spec

    spec = BlenderSceneSpec()                           # 默认: 程序化场景
    spec = BlenderSceneSpec(scene_xml_path="export.xml") # Mitsuba 直出
    spec = BlenderSceneSpec(ply_dir="./meshes", material_map_path="./materials.json")
    scene = load_scene_from_spec(spec)
"""

from __future__ import annotations

import os
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

import sionna
from sionna.rt import load_scene, PlanarArray, Transmitter, Receiver


# ── 场景默认配置 ──────────────────────────────────────────────────────

FREQ_35G = 3.5e9
FREQ_28G = 28e9

BS_POSITIONS = [
    ("bs_0", [20.0, 20.0, 8.0]),
    ("bs_1", [-20.0, -20.0, 8.0]),
    ("bs_2", [20.0, -20.0, 8.0]),
]

DRONE_POS = [0.0, 0.0, 3.0]
MAP_CENTER = [0.0, 0.0, 2.0]
MAP_SIZE = [60.0, 60.0]
MAP_CELL_SIZES = {"3.5GHz": (2.0, 2.0), "28GHz": (1.0, 1.0)}
RT_MAX_DEPTH = 5
RT_SAMPLES_PER_TX = 5_000_000

# 支持的材料列表
ITU_MATERIALS = {
    "metal", "wood", "concrete", "brick", "glass",
    "plasterboard", "marble", "ceiling_board", "fiber_glass",
    "tiles", "floorboard",
    "very_dry_ground", "medium_dry_ground", "wet_ground",
}


# ── BlenderSceneSpec ──────────────────────────────────────────────────

@dataclass
class BlenderSceneSpec:
    """
    Blender 场景规范 — Blender 导出与 Sionna RT 之间的契约。

    两种 Blender 导入方式（互斥，优先使用 scene_xml_path）：
      1. scene_xml_path: Mitsuba XML 直出（mitsuba-blender 插件导出）
      2. ply_dir + material_map_path: PLY 中转（需配合 generate_scene_xml.py）

    若两者均未提供，自动使用程序化场景回退。

    Parameters
    ----------
    scene_xml_path : str or None
        Mitsuba XML 场景文件路径（方式1: mitsuba-blender 直出）。
    ply_dir : str or None
        PLY 网格文件目录（方式2: Blender PLY 导出）。
    material_map_path : str or None
        材质映射 JSON 文件路径（方式2 必填）。
    scene_name : str
        场景名称标识。
    bs_positions : list of (str, [float, float, float])
        基站列表 (名称, 位置 [x,y,z] in meters)。
    drone_position : [float, float, float]
        无人机接收机初始位置 [x,y,z] in meters。
    map_center : [float, float, float]
        无线电地图测量平面中心 [x,y,z] in meters。
    map_size : [float, float]
        无线电地图尺寸 (width, height) in meters。
    cell_sizes : dict
        各频段 cell_size，如 {"3.5GHz": (2,2), "28GHz": (1,1)}。
    max_depth : int
        射线追踪最大弹跳次数。
    samples_per_tx : int
        RadioMapSolver 每发射机采样数。
    """

    # ── Blender 导出路径（二选一）──
    scene_xml_path: Optional[str] = None       # 方式1: Mitsuba XML 直出
    ply_dir: Optional[str] = None              # 方式2: PLY 网格目录
    material_map_path: Optional[str] = None    # 方式2: 材质映射 JSON

    # ── 场景元数据 ──
    scene_name: str = "drone_scene"
    bs_positions: List[Tuple[str, List[float]]] = field(
        default_factory=lambda: [list(x) for x in BS_POSITIONS]
    )
    drone_position: List[float] = field(default_factory=lambda: list(DRONE_POS))
    map_center: List[float] = field(default_factory=lambda: list(MAP_CENTER))
    map_size: List[float] = field(default_factory=lambda: list(MAP_SIZE))
    cell_sizes: dict = field(default_factory=lambda: dict(MAP_CELL_SIZES))
    max_depth: int = RT_MAX_DEPTH
    samples_per_tx: int = RT_SAMPLES_PER_TX

    def __post_init__(self):
        # 确保 bs_positions 是 list of (str, list)
        self.bs_positions = [
            (name, list(pos)) for name, pos in self.bs_positions
        ]


# ── 验证函数 ───────────────────────────────────────────────────────────

def validate_blender_export(spec: BlenderSceneSpec) -> List[str]:
    """
    验证 Blender 导出文件完整性。

    Returns
    -------
    list[str]
        警告/错误信息列表。空列表 = 通过验证。
    """
    issues = []

    if spec.scene_xml_path is not None:
        # 方式1: Mitsuba XML 直出
        xml_path = Path(spec.scene_xml_path)
        if not xml_path.exists():
            issues.append(f"[ERROR] scene_xml_path 不存在: {spec.scene_xml_path}")
        elif not xml_path.suffix.lower() in (".xml",):
            issues.append(f"[WARN] XML 文件扩展名非 .xml: {spec.scene_xml_path}")
        return issues

    if spec.ply_dir is not None:
        # 方式2: PLY 中转
        ply_path = Path(spec.ply_dir)
        if not ply_path.exists():
            issues.append(f"[ERROR] ply_dir 不存在: {spec.ply_dir}")
            return issues
        if not ply_path.is_dir():
            issues.append(f"[ERROR] ply_dir 不是目录: {spec.ply_dir}")
            return issues

        ply_files = list(ply_path.glob("*.ply"))
        if not ply_files:
            issues.append(f"[WARN] ply_dir 中没有 .ply 文件: {spec.ply_dir}")

        if spec.material_map_path is None:
            issues.append("[WARN] ply_dir 已指定但 material_map_path 未指定，将使用默认材质")
        elif not Path(spec.material_map_path).exists():
            issues.append(f"[ERROR] material_map_path 不存在: {spec.material_map_path}")

        return issues

    # 双方都未提供 → 使用程序化回退（合法情况，不报错）
    return issues


# ── 程序化场景生成（回退方案）──────────────────────────────────────────

def _ply_header(n_vertices: int, n_faces: int) -> str:
    return (
        "ply\n"
        "format ascii 1.0\n"
        f"element vertex {n_vertices}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        f"element face {n_faces}\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    )


def make_box_ply(width: float, depth: float, height: float) -> str:
    """生成立方体 PLY 内容字符串"""
    w, d, h = width / 2, depth / 2, height / 2
    verts = [
        [-w, -d, -h], [w, -d, -h], [w, d, -h], [-w, d, -h],
        [-w, -d, h], [w, -d, h], [w, d, h], [-w, d, h],
    ]
    faces = [
        [0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
        [0, 4, 5], [0, 5, 1], [2, 6, 7], [2, 7, 3],
        [1, 5, 6], [1, 6, 2], [3, 7, 4], [3, 4, 0],
    ]
    lines = [_ply_header(8, 12)]
    for v in verts:
        lines.append(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}")
    for f in faces:
        lines.append(f"3 {f[0]} {f[1]} {f[2]}")
    return "\n".join(lines) + "\n"


def make_cylinder_ply(radius: float, height: float, n_sides: int = 12) -> str:
    """生成圆柱体 PLY 内容字符串（沿 Z 轴）"""
    h = height / 2
    verts = [[0.0, 0.0, -h], [0.0, 0.0, h]]  # 0: bottom center, 1: top center
    for i in range(n_sides):
        angle = 2 * np.pi * i / n_sides
        verts.append([radius * np.cos(angle), radius * np.sin(angle), -h])
        verts.append([radius * np.cos(angle), radius * np.sin(angle), h])

    faces = []
    for i in range(n_sides):
        j = (i + 1) % n_sides
        b0, b1 = 2 + i, 2 + j
        t0, t1 = 2 + n_sides + i, 2 + n_sides + j
        faces.append([b0, b1, t1])
        faces.append([b0, t1, t0])
        faces.append([0, 2 + i, 2 + j])
        faces.append([1, 2 + n_sides + j, 2 + n_sides + i])

    n_verts = 2 + 2 * n_sides
    n_faces = 4 * n_sides
    lines = [_ply_header(n_verts, n_faces)]
    for v in verts:
        lines.append(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}")
    for f in faces:
        lines.append(f"3 {f[0]} {f[1]} {f[2]}")
    return "\n".join(lines) + "\n"


def make_cylinder_ply_along_axis(
    start: tuple, end: tuple, radius: float, n_sides: int = 12
) -> str:
    """生成沿任意轴方向的圆柱体 PLY"""
    start, end = np.array(start), np.array(end)
    axis = end - start
    length = np.linalg.norm(axis)
    axis = axis / length

    if abs(axis[0]) < 0.9:
        u = np.cross(axis, [1, 0, 0])
    else:
        u = np.cross(axis, [0, 1, 0])
    u = u / np.linalg.norm(u)
    v = np.cross(axis, u)

    verts = [start, end]
    for i in range(n_sides):
        angle = 2 * np.pi * i / n_sides
        offset = radius * (np.cos(angle) * u + np.sin(angle) * v)
        verts.append(start + offset)
        verts.append(end + offset)

    faces = []
    for i in range(n_sides):
        j = (i + 1) % n_sides
        s0, s1 = 2 + i, 2 + j
        e0, e1 = 2 + n_sides + i, 2 + n_sides + j
        faces.append([s0, s1, e1])
        faces.append([s0, e1, e0])
        faces.append([0, s1, s0])
        faces.append([1, e0, e1])

    n_verts = 2 + 2 * n_sides
    n_faces = 4 * n_sides
    lines = [_ply_header(n_verts, n_faces)]
    for vt in verts:
        lines.append(f"{vt[0]:.6f} {vt[1]:.6f} {vt[2]:.6f}")
    for f in faces:
        lines.append(f"3 {f[0]} {f[1]} {f[2]}")
    return "\n".join(lines) + "\n"


def create_procedural_scene(spec: BlenderSceneSpec, tmpdir: str) -> str:
    """
    生成程序化无人机场景（四旋翼模型 + 地面 + 3 个基站）。

    无人机模型组成：
      - 1 个中心机身 box（0.30 x 0.30 x 0.06 m）
      - 4 个水平臂圆柱（臂长 0.35 m，半径 0.015 m）
      - 4 个垂直电机圆柱（半径 0.03 m，高 0.03 m）
      - 1 个大型地面平面（80 x 80 m）

    Returns
    -------
    str
        生成 Mitsuba XML 文件路径。
    """
    x, y, z = spec.drone_position
    ply_dir = os.path.join(tmpdir, "meshes")
    xml_path = os.path.join(tmpdir, "drone_scene.xml")
    os.makedirs(ply_dir, exist_ok=True)

    box_w, box_d, box_h = 0.30, 0.30, 0.06
    arm_len = 0.35
    arm_radius = 0.015
    motor_radius = 0.03
    motor_height = 0.03
    ground_size = 80.0

    # 地面
    ground_ply = os.path.join(ply_dir, "ground.ply")
    vertices = [
        [-ground_size / 2, -ground_size / 2, 0.0],
        [ground_size / 2, -ground_size / 2, 0.0],
        [ground_size / 2, ground_size / 2, 0.0],
        [-ground_size / 2, ground_size / 2, 0.0],
    ]
    with open(ground_ply, "w") as f:
        f.write(_ply_header(4, 2))
        for v in vertices:
            f.write(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        f.write("3 0 1 2\n3 0 2 3\n")

    # 机身
    body_ply = os.path.join(ply_dir, "drone_body.ply")
    with open(body_ply, "w") as f:
        f.write(make_box_ply(box_w, box_d, box_h))

    # 4 个臂
    arm_dirs = [
        (box_w / 2, box_d / 2), (box_w / 2, -box_d / 2),
        (-box_w / 2, box_d / 2), (-box_w / 2, -box_d / 2),
    ]
    for i, (dx, dy) in enumerate(arm_dirs):
        arm_start = (dx, dy, 0.0)
        arm_end = (dx + np.sign(dx) * arm_len, dy + np.sign(dy) * arm_len, 0.0)
        arm_ply = os.path.join(ply_dir, f"drone_arm_{i+1}.ply")
        with open(arm_ply, "w") as f:
            f.write(make_cylinder_ply_along_axis(arm_start, arm_end, arm_radius))

    # 4 个电机
    for i, (dx, dy) in enumerate(arm_dirs):
        motor_ply = os.path.join(ply_dir, f"drone_motor_{i+1}.ply")
        with open(motor_ply, "w") as f:
            f.write(make_cylinder_ply(motor_radius, motor_height))

    # 生成 XML
    xml_lines = [
        '<scene version="2.1.0">', '',
        '<!-- Materials -->',
        '    <bsdf type="itu-radio-material" id="metal">',
        '        <string name="type" value="metal"/>',
        '        <float name="thickness" value="0.02"/>',
        '    </bsdf>',
        '    <bsdf type="itu-radio-material" id="concrete">',
        '        <string name="type" value="concrete"/>',
        '        <float name="thickness" value="0.3"/>',
        '    </bsdf>',
        '',
        '<!-- Shapes -->',
        f'    <shape type="ply" id="mesh-ground">',
        f'        <string name="filename" value="{ply_dir}/ground.ply"/>',
        f'        <ref id="concrete" name="bsdf"/>',
        f'    </shape>',
        f'    <shape type="ply" id="mesh-body">',
        f'        <string name="filename" value="{ply_dir}/drone_body.ply"/>',
        f'        <ref id="metal" name="bsdf"/>',
        f'    </shape>',
    ]
    for i in range(1, 5):
        for part in ("arm", "motor"):
            xml_lines.append(
                f'    <shape type="ply" id="mesh-{part}{i}">\n'
                f'        <string name="filename" value="{ply_dir}/drone_{part}_{i}.ply"/>\n'
                f'        <ref id="metal" name="bsdf"/>\n'
                f'    </shape>'
            )
    xml_lines.append('</scene>')

    with open(xml_path, "w") as f:
        f.write("\n".join(xml_lines))

    print(f"  [程序化场景] XML: {xml_path}")
    print(f"  [程序化场景] PLY: {ply_dir} ({len(list(Path(ply_dir).glob('*.ply')))} 个网格)")
    return xml_path


# ── 场景加载 ────────────────────────────────────────────────────────────

def load_scene_from_spec(spec: BlenderSceneSpec, _tmpdir: Optional[str] = None):
    """
    根据 BlenderSceneSpec 加载 Sionna RT 场景对象。

    优先级:
      1. spec.scene_xml_path → Mitsuba XML 直出直接加载
      2. spec.ply_dir → 调用 generate_scene_xml.py → 加载
      3. 以上皆无 → 程序化场景生成

    Parameters
    ----------
    spec : BlenderSceneSpec
        场景规范。
    _tmpdir : str or None
        程序化场景使用的临时目录。None 则自动创建 TemporaryDirectory。
        注意：调用方需管理此目录的生命周期。

    Returns
    -------
    scene : sionna.rt.Scene
        已加载天线阵列的 Sionna RT 场景对象。
    tmpdir : str or None
        程序化场景的临时目录（仅当使用回退方案时非 None）。
    """
    tmpdir = None

    # 路径 1: Mitsuba XML 直出
    if spec.scene_xml_path is not None:
        xml_path = spec.scene_xml_path
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"scene_xml_path 不存在: {xml_path}")
        print(f"  [加载] Mitsuba XML 直出: {xml_path}")
        source = "mitsuba-blender 直出"

    # 路径 2: PLY 中转
    elif spec.ply_dir is not None:
        if not os.path.isdir(spec.ply_dir):
            raise FileNotFoundError(f"ply_dir 不存在或不是目录: {spec.ply_dir}")

        # 将 generate_scene_xml 所在目录加入 sys.path
        tools_dir = os.path.join(os.path.dirname(__file__), "..", "tools", "blender_to_sionna")
        tools_dir = os.path.abspath(tools_dir)
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)

        from generate_scene_xml import generate_xml

        tmpdir = tempfile.mkdtemp(prefix="sionna_ply_")
        xml_path = os.path.join(tmpdir, "scene.xml")
        xml_content = generate_xml(
            ply_dir=spec.ply_dir,
            material_map_path=spec.material_map_path or "",
            scene_name=spec.scene_name,
        )
        with open(xml_path, "w") as f:
            f.write(xml_content)
        print(f"  [加载] PLY 中转 → XML: {xml_path}")
        source = "PLY 中转 (Blender 导出)"

    # 路径 3: 程序化回退
    else:
        tmpdir = tempfile.mkdtemp(prefix="sionna_proc_")
        xml_path = create_procedural_scene(spec, tmpdir)
        print(f"  [加载] 程序化场景回退")
        source = "程序化生成 (Blender 未安装)"

    # 加载场景
    scene = load_scene(xml_path)
    scene.tx_array = PlanarArray(
        num_rows=1, num_cols=1,
        vertical_spacing=0.5, horizontal_spacing=0.5,
        pattern="iso", polarization="V",
    )
    scene.rx_array = PlanarArray(
        num_rows=1, num_cols=1,
        vertical_spacing=0.5, horizontal_spacing=0.5,
        pattern="iso", polarization="V",
    )

    # 添加设备
    for name, pos in spec.bs_positions:
        tx = Transmitter(name=name, position=pos, orientation=[0, 0, 0],
                         power_dbm=44)
        scene.add(tx)

    rx = Receiver(name="drone_rx", position=spec.drone_position,
                  orientation=[0, 0, 0])
    scene.add(rx)

    print(f"  场景来源: {source}")
    print(f"  场景物体: {list(scene.scene_objects.keys())}")
    print(f"  发射机:   {list(scene.transmitters.keys())}")
    print(f"  接收机:   {list(scene.receivers.keys())}")

    return scene, tmpdir


def scene_summary(spec: BlenderSceneSpec, scene) -> None:
    """打印场景配置的格式化摘要"""
    print("\n" + "=" * 60)
    print(f"  场景名称: {spec.scene_name}")
    print(f"  场景物体: {list(scene.scene_objects.keys())}")
    print(f"  发射机数量: {len(scene.transmitters)}")
    for name, tx in scene.transmitters.items():
        pos = tx.position.numpy() if hasattr(tx.position, 'numpy') else tx.position
        print(f"    - {name}: pos={pos}")
    print(f"  接收机数量: {len(scene.receivers)}")
    for name, rx in scene.receivers.items():
        pos = rx.position.numpy() if hasattr(rx.position, 'numpy') else rx.position
        print(f"    - {name}: pos={pos}")
    print(f"  地图中心: {spec.map_center}")
    print(f"  地图尺寸: {spec.map_size}")
    print(f"  RT 最大深度: {spec.max_depth}")
    print(f"  RT 采样数/TX: {spec.samples_per_tx:,}")
    print("=" * 60 + "\n")


# ── 自测 ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("Step 8 自测: Blender 场景接口\n")

    # 测试 1: 默认程序化场景
    spec = BlenderSceneSpec()
    issues = validate_blender_export(spec)
    print(f"  验证结果: {issues if issues else '通过'}")

    scene, tmpdir = load_scene_from_spec(spec)
    scene_summary(spec, scene)
    print("  自测通过!\n")

    # 测试 2: 验证 XML 路径（不存在应报 WARN）
    spec2 = BlenderSceneSpec(scene_xml_path="/nonexistent/scene.xml")
    issues2 = validate_blender_export(spec2)
    print(f"  验证结果（预期 WARN）: {issues2}")

    # 清理
    import shutil
    if tmpdir and os.path.exists(tmpdir):
        shutil.rmtree(tmpdir)
