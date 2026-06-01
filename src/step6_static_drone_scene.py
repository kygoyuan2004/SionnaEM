#!/usr/bin/env python3
"""
Step 6: 静态无人机场景 — 多基站信道图与路径损耗图

在 Sionna RT 场景中放置简易无人机模型，配置 3 个基站，
分别用 3.5 GHz 和 28 GHz 两个频段生成信道图和路径损耗图。

验证目标:
  1. 各基站信道图可独立拆分 (Q1)
  2. 双频段对比 (Q5)
  3. PathSolver + RadioMapSolver 完整管线

用法:
    cd src
    python step6_static_drone_scene.py                    # 默认: floor_wall 场景, 两种频段
    python step6_static_drone_scene.py --freq 3.5         # 仅 3.5 GHz
    python step6_static_drone_scene.py --freq 28          # 仅 28 GHz
    python step6_static_drone_scene.py --scene etoile     # 巴黎场景
"""

import argparse
import os
import sys
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import sionna
from sionna.rt import (
    load_scene, PlanarArray, Transmitter, Receiver,
    PathSolver, RadioMapSolver, Camera,
    SceneObject,
)
from sionna.rt.radio_materials import ITURadioMaterial

# ── 配置 ──────────────────────────────────────────────────────────

# 两个目标频段
FREQ_35G = 3.5e9
FREQ_28G = 28e9

# 3 个基站位置（三角分布）
BS_POSITIONS = [
    ("bs_0", [20.0, 20.0, 8.0]),
    ("bs_1", [-20.0, -20.0, 8.0]),
    ("bs_2", [20.0, -20.0, 8.0]),
]

# 无人机初始位置（空中 3m）
DRONE_POS = [0.0, 0.0, 3.0]

# 无线电地图测量平面
MAP_CENTER = [0.0, 0.0, 2.0]  # 略低于无人机高度
MAP_SIZE = [60.0, 60.0]
MAP_CELL_SIZES = {
    "3.5GHz": (2.0, 2.0),
    "28GHz": (1.0, 1.0),
}

# 射线追踪参数
RT_MAX_DEPTH = 5
RT_SAMPLES_PER_TX = 5_000_000  # 每个 TX 的蒙特卡洛采样


# ── 简易 PLY 几何体生成 ──────────────────────────────────────────

def _ply_header(n_vertices, n_faces):
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
        [0, 1, 2], [0, 2, 3],  # bottom
        [4, 6, 5], [4, 7, 6],  # top
        [0, 4, 5], [0, 5, 1],  # front
        [2, 6, 7], [2, 7, 3],  # back
        [1, 5, 6], [1, 6, 2],  # right
        [3, 7, 4], [3, 4, 0],  # left
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
    verts = []
    # 顶面和底面圆心
    verts.append([0.0, 0.0, -h])  # 0: bottom center
    verts.append([0.0, 0.0, h])   # 1: top center
    for i in range(n_sides):
        angle = 2 * np.pi * i / n_sides
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        verts.append([x, y, -h])  # bottom ring: 2..2+n_sides-1
        verts.append([x, y, h])   # top ring: 2+n_sides..2+2*n_sides-1

    faces = []
    # 侧面
    for i in range(n_sides):
        j = (i + 1) % n_sides
        b0 = 2 + i
        b1 = 2 + j
        t0 = 2 + n_sides + i
        t1 = 2 + n_sides + j
        faces.append([b0, b1, t1])
        faces.append([b0, t1, t0])
    # 底面
    for i in range(n_sides):
        j = (i + 1) % n_sides
        faces.append([0, 2 + i, 2 + j])
    # 顶面
    for i in range(n_sides):
        j = (i + 1) % n_sides
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
    """生成沿任意轴方向的圆柱体 PLY（用于无人机臂）"""
    start, end = np.array(start), np.array(end)
    axis = end - start
    length = np.linalg.norm(axis)
    axis = axis / length

    # 构建正交基
    if abs(axis[0]) < 0.9:
        u = np.cross(axis, [1, 0, 0])
    else:
        u = np.cross(axis, [0, 1, 0])
    u = u / np.linalg.norm(u)
    v = np.cross(axis, u)

    verts = [start, end]  # 0: start, 1: end
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


# ── 无人机场景 XML 生成 ───────────────────────────────────────────

def generate_drone_scene_xml(ply_dir: str, drone_pos: tuple) -> str:
    """
    生成包含四旋翼无人机和地面的场景 XML。
    无人机 = 1 个中心机身 box + 4 个水平臂圆柱 + 4 个垂直电机小圆柱
    """
    x, y, z = drone_pos
    box_w, box_d, box_h = 0.30, 0.30, 0.06   # 机身
    arm_len = 0.35
    arm_radius = 0.015
    motor_radius = 0.03
    motor_height = 0.03
    ground_size = 80.0

    # ── 生成 PLY 文件 ──
    os.makedirs(ply_dir, exist_ok=True)

    # 地面
    ground_ply = os.path.join(ply_dir, "ground.ply")
    # 用大扁 box 做地面
    vertices = [
        [-ground_size / 2, -ground_size / 2, 0.0],
        [ground_size / 2, -ground_size / 2, 0.0],
        [ground_size / 2, ground_size / 2, 0.0],
        [-ground_size / 2, ground_size / 2, 0.0],
    ]
    faces = [[0, 1, 2], [0, 2, 3]]
    with open(ground_ply, "w") as f:
        f.write(_ply_header(4, 2))
        for v in vertices:
            f.write(f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for face in faces:
            f.write(f"3 {face[0]} {face[1]} {face[2]}\n")

    # 机身
    body_ply = os.path.join(ply_dir, "drone_body.ply")
    with open(body_ply, "w") as f:
        f.write(make_box_ply(box_w, box_d, box_h))

    # 4 个臂（从中心向对角线延伸）
    arm_dirs = [
        (box_w / 2, box_d / 2),
        (box_w / 2, -box_d / 2),
        (-box_w / 2, box_d / 2),
        (-box_w / 2, -box_d / 2),
    ]
    for i, (dx, dy) in enumerate(arm_dirs):
        arm_start = (dx, dy, 0.0)
        arm_end = (dx + np.sign(dx) * arm_len, dy + np.sign(dy) * arm_len, 0.0)
        arm_ply = os.path.join(ply_dir, f"drone_arm_{i+1}.ply")
        with open(arm_ply, "w") as f:
            f.write(make_cylinder_ply_along_axis(arm_start, arm_end, arm_radius))

    # 4 个电机（在臂的末端）
    for i, (dx, dy) in enumerate(arm_dirs):
        motor_x = dx + np.sign(dx) * arm_len
        motor_y = dy + np.sign(dy) * arm_len
        motor_ply = os.path.join(ply_dir, f"drone_motor_{i+1}.ply")
        with open(motor_ply, "w") as f:
            f.write(make_cylinder_ply(motor_radius, motor_height))

    # ── 生成 XML ──
    mat_entries = [
        ('    <bsdf type="itu-radio-material" id="metal">\n'
         '        <string name="type" value="metal"/>\n'
         '        <float name="thickness" value="0.02"/>\n'
         '    </bsdf>'),
        ('    <bsdf type="itu-radio-material" id="concrete">\n'
         '        <string name="type" value="concrete"/>\n'
         '        <float name="thickness" value="0.3"/>\n'
         '    </bsdf>'),
    ]

    shapes = [
        f'    <shape type="ply" id="mesh-ground">\n'
        f'        <string name="filename" value="{ply_dir}/ground.ply"/>\n'
        f'        <ref id="concrete" name="bsdf"/>\n'
        f'    </shape>',
        f'    <shape type="ply" id="mesh-body">\n'
        f'        <string name="filename" value="{ply_dir}/drone_body.ply"/>\n'
        f'        <ref id="metal" name="bsdf"/>\n'
        f'    </shape>',
    ]
    for i in range(1, 5):
        for part in ("arm", "motor"):
            shapes.append(
                f'    <shape type="ply" id="mesh-{part}{i}">\n'
                f'        <string name="filename" value="{ply_dir}/drone_{part}_{i}.ply"/>\n'
                f'        <ref id="metal" name="bsdf"/>\n'
                f'    </shape>'
            )

    xml = ['<scene version="2.1.0">', '', '<!-- Materials -->']
    xml.extend(mat_entries)
    xml.append('')
    xml.append('<!-- Shapes -->')
    xml.extend(shapes)
    xml.append('</scene>')
    return "\n".join(xml)


# ── 无线电地图计算与可视化 ──────────────────────────────────────

def create_base_stations(scene, power_dbm=44):
    """在场景中放置 3 个基站"""
    for name, pos in BS_POSITIONS:
        tx = Transmitter(name=name, position=pos, orientation=[0, 0, 0],
                         power_dbm=power_dbm)
        scene.add(tx)
    print(f"  已添加 {len(BS_POSITIONS)} 个基站发射机")


def compute_and_visualize(scene, freq_label, freq_hz, cell_size, output_dir):
    """计算无线电地图并生成可视化"""
    scene.frequency = freq_hz

    print(f"\n  [{freq_label}] 场景频率 = {freq_hz/1e9:.1f} GHz, "
          f"波长 = {scene.wavelength.numpy()[0]*1e3:.2f} mm")

    # PathSolver
    print(f"  [{freq_label}] 计算传播路径...")
    solver = PathSolver()
    paths = solver(scene, max_depth=RT_MAX_DEPTH, los=True,
                   specular_reflection=True, diffuse_reflection=False,
                   diffraction=False, edge_diffraction=False,
                   refraction=False, seed=42)

    # 统计每条 TX-RX 对的路径数
    path_counts = []
    for tx_name, _ in BS_POSITIONS:
        tx_idx = list(scene.transmitters.keys()).index(tx_name)
        n_paths = paths.a.shape[-1]  # 取最后一个维度的路径数
        path_counts.append(n_paths)

    print(f"  [{freq_label}] 各 BS-RX 路径数: {path_counts}")

    # RadioMapSolver
    print(f"  [{freq_label}] 计算无线电地图 (cell={cell_size}, "
          f"samples_per_tx={RT_SAMPLES_PER_TX})...")
    rm_solver = RadioMapSolver()
    radio_map = rm_solver(
        scene,
        center=MAP_CENTER,
        orientation=[0, 0, 0],
        size=MAP_SIZE,
        cell_size=cell_size,
        samples_per_tx=RT_SAMPLES_PER_TX,
        max_depth=RT_MAX_DEPTH,
        los=True, specular_reflection=True,
        diffuse_reflection=False, diffraction=False,
        seed=42,
    )

    pg = radio_map.path_gain.numpy()  # [num_tx, cells_y, cells_x]
    pg_db = 10 * np.log10(pg + 1e-15)
    num_tx = pg.shape[0]

    print(f"  [{freq_label}] RadioMap path_gain shape: {pg.shape}")
    for i in range(num_tx):
        print(f"    BS {i} ({list(scene.transmitters.keys())[i]}): "
              f"path_gain range = [{pg_db[i].min():.1f}, {pg_db[i].max():.1f}] dB")

    # ── 可视化 ──
    fig, axes = plt.subplots(2, num_tx + 1,
                              figsize=(5 * (num_tx + 1), 10),
                              constrained_layout=True)

    # 预计算全局 colorbar 范围
    all_valid = pg_db[pg_db > -200]
    vmin = np.percentile(all_valid, 2) if len(all_valid) > 0 else -160
    vmax = np.percentile(all_valid, 98) if len(all_valid) > 0 else -60

    # Row 0: 各基站独立图
    for i in range(num_tx):
        ax = axes[0, i]
        im = ax.imshow(pg_db[i], origin="lower", cmap="jet",
                       vmin=vmin, vmax=vmax,
                       extent=[-MAP_SIZE[0]/2, MAP_SIZE[0]/2,
                               -MAP_SIZE[1]/2, MAP_SIZE[1]/2])
        ax.set_title(f"BS {i}: {list(scene.transmitters.keys())[i]}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        # 标注无人机位置
        ax.plot(DRONE_POS[0], DRONE_POS[1], "wx", markersize=10, markeredgewidth=2)
        # 标注当前 BS 位置
        bs_pos = BS_POSITIONS[i][1]
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)

    # 第 n+1 列: 合成图 (max over all BS)
    ax = axes[0, num_tx]
    combined = pg_db.max(axis=0)
    im = ax.imshow(combined, origin="lower", cmap="jet",
                   vmin=vmin, vmax=vmax,
                   extent=[-MAP_SIZE[0]/2, MAP_SIZE[0]/2,
                           -MAP_SIZE[1]/2, MAP_SIZE[1]/2])
    ax.set_title("Combined (max over BS)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.plot(DRONE_POS[0], DRONE_POS[1], "wx", markersize=10, markeredgewidth=2)
    for _, bs_pos in BS_POSITIONS:
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)
    plt.colorbar(im, ax=axes[0, :].tolist(), label="Path Gain [dB]", shrink=0.6)

    # Row 1: RSS map (含发射功率)
    rss = radio_map.rss.numpy()  # [num_tx, cells_y, cells_x]
    for i in range(num_tx):
        ax = axes[1, i]
        im = ax.imshow(rss[i], origin="lower", cmap="jet",
                       extent=[-MAP_SIZE[0]/2, MAP_SIZE[0]/2,
                               -MAP_SIZE[1]/2, MAP_SIZE[1]/2])
        ax.set_title(f"RSS BS {i}: {list(scene.transmitters.keys())[i]}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.plot(DRONE_POS[0], DRONE_POS[1], "wx", markersize=10, markeredgewidth=2)
        bs_pos = BS_POSITIONS[i][1]
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)

    ax = axes[1, num_tx]
    rss_combined = rss.max(axis=0)
    im = ax.imshow(rss_combined, origin="lower", cmap="jet",
                   extent=[-MAP_SIZE[0]/2, MAP_SIZE[0]/2,
                           -MAP_SIZE[1]/2, MAP_SIZE[1]/2])
    ax.set_title("RSS Combined (max over BS)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.plot(DRONE_POS[0], DRONE_POS[1], "wx", markersize=10, markeredgewidth=2)
    for _, bs_pos in BS_POSITIONS:
        ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)
    plt.colorbar(im, ax=axes[1, :].tolist(), label="RSS [dBm]", shrink=0.6)

    fig.suptitle(f"Drone Scene Radio Maps — {freq_label} ({freq_hz/1e9:.1f} GHz)\n"
                 f"Drone @ {DRONE_POS}, {num_tx} Base Stations",
                 fontsize=14, fontweight="bold")

    fig_path = os.path.join(output_dir, f"step6_static_drone_{freq_label.replace('.', 'p')}.png")
    fig.savefig(fig_path, dpi=150, bbox_inches="tight")
    print(f"  [{freq_label}] 保存: {fig_path}")

    return radio_map, paths


def run(args):
    """主流程"""
    output_dir = args.output_dir or "../figures"
    os.makedirs(output_dir, exist_ok=True)

    # 使用临时目录存放生成的 PLY 和 XML
    with tempfile.TemporaryDirectory() as tmpdir:
        ply_dir = os.path.join(tmpdir, "meshes")
        xml_path = os.path.join(tmpdir, "drone_scene.xml")

        # Step 1: 生成无人机场景
        print("=" * 60)
        print("Step 6: 静态无人机场景 — 多基站信道图")
        print("=" * 60)

        print("\n[1] 生成程序化无人机场景...")
        xml_content = generate_drone_scene_xml(ply_dir, DRONE_POS)
        with open(xml_path, "w") as f:
            f.write(xml_content)
        print(f"  场景 XML: {xml_path}")
        print(f"  PLY 目录: {ply_dir}")

        # 列出生成的文件
        ply_files = list(Path(ply_dir).glob("*.ply"))
        print(f"  已生成 {len(ply_files)} 个 PLY 网格:")
        for pf in sorted(ply_files):
            print(f"    - {pf.name}")

        # Step 2: 加载场景
        print("\n[2] 加载场景到 Sionna RT...")
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
        print(f"  场景对象: {list(scene.scene_objects.keys())}")

        # Step 3: 添加基站和接收机
        print("\n[3] 添加基站和无人机接收机...")
        create_base_stations(scene, power_dbm=44)
        rx = Receiver(name="drone_rx", position=DRONE_POS, orientation=[0, 0, 0])
        scene.add(rx)
        print(f"  接收机位置: {DRONE_POS}")

        # Step 4: 分别用两个频段计算
        results = {}
        freqs_to_run = []
        if args.freq in ("3.5", "all"):
            freqs_to_run.append(("3.5GHz", FREQ_35G, MAP_CELL_SIZES["3.5GHz"]))
        if args.freq in ("28", "all"):
            freqs_to_run.append(("28GHz", FREQ_28G, MAP_CELL_SIZES["28GHz"]))

        for freq_label, freq_hz, cell_size in freqs_to_run:
            print(f"\n[4] 计算 {freq_label}...")
            print("-" * 40)
            rm, paths = compute_and_visualize(
                scene, freq_label, freq_hz, cell_size, output_dir
            )
            results[freq_label] = {"radio_map": rm, "paths": paths}

    # Step 5: 如果两个频段都计算了，生成对比图
    if len(results) == 2:
        print("\n[5] 生成双频段对比图...")
        fig, axes = plt.subplots(1, 2, figsize=(16, 7), constrained_layout=True)

        for ax, (label, data) in zip(axes, results.items()):
            pg = data["radio_map"].path_gain.numpy()
            pg_db = 10 * np.log10(pg.max(axis=0) + 1e-15)
            im = ax.imshow(pg_db, origin="lower", cmap="jet",
                           extent=[-MAP_SIZE[0]/2, MAP_SIZE[0]/2,
                                   -MAP_SIZE[1]/2, MAP_SIZE[1]/2])
            ax.set_title(f"{label} — Combined Path Gain", fontsize=13)
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.plot(DRONE_POS[0], DRONE_POS[1], "wx", markersize=10, markeredgewidth=2)
            for _, bs_pos in BS_POSITIONS:
                ax.plot(bs_pos[0], bs_pos[1], "k^", markersize=12, markeredgewidth=2)
            plt.colorbar(im, ax=ax, label="Path Gain [dB]", shrink=0.8)

        fig.suptitle("Dual-Band Comparison: 3.5 GHz vs 28 GHz\n"
                     f"Drone @ {DRONE_POS}, 3 Base Stations, max_depth={RT_MAX_DEPTH}",
                     fontsize=14, fontweight="bold")
        cmp_path = os.path.join(output_dir, "step6_dual_band_comparison.png")
        fig.savefig(cmp_path, dpi=150, bbox_inches="tight")
        print(f"  保存: {cmp_path}")

    print("\n" + "=" * 60)
    print("Step 6 完成!")
    print(f"输出目录: {output_dir}")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Step 6: 静态无人机场景")
    parser.add_argument("--freq", default="all", choices=["3.5", "28", "all"],
                        help="频段选择 (default: all)")
    parser.add_argument("--output-dir", default=None,
                        help="输出目录 (default: ../figures)")
    args = parser.parse_args()
    run(args)
