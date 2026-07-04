#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
物流技术创意赛 — 简化比赛场地 USD 场景生成脚本

用途：在 Isaac Sim 的 Script Editor 或 standalone Python 环境中运行，
      生成包含场地几何、简化机器人和基础传感器挂载点的 USD 场景。

坐标约定：
  - 单位：米 (m)
  - 场地中心为世界原点 (0, 0, 0)
  - x 轴：场地长度方向，约 -4.0 ~ 4.0
  - y 轴：场地宽度方向，约 -2.0 ~ 2.0
  - z 轴：向上；世界地面 z = 0，主场地地板上表面 z = 0.007
"""

from __future__ import annotations

import os
import random
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ---------------------------------------------------------------------------
# 集中参数 — 修改布局时优先改这里
# ---------------------------------------------------------------------------

FIELD_SCALE = 2.0             # 场地和场地物体同比放大倍数；机器人模型保持原尺寸

# 场地地面：原 4m × 2m × 3.5mm，按 FIELD_SCALE 放大，底面直接放在世界地面 z=0 上
FIELD_LENGTH = 4.0 * FIELD_SCALE          # x 方向长度 (m)
FIELD_WIDTH = 2.0 * FIELD_SCALE           # y 方向宽度 (m)
FIELD_THICKNESS = 0.0035 * FIELD_SCALE    # 地面厚度 (m)
FIELD_BOTTOM_Z = 0.0          # 主场地底面贴世界地面
FIELD_CENTER_Z = FIELD_BOTTOM_Z + FIELD_THICKNESS / 2.0
FIELD_TOP_Z = FIELD_BOTTOM_Z + FIELD_THICKNESS

# 围栏
FENCE_HEIGHT = 0.10 * FIELD_SCALE           # 围栏高度 (m)
FENCE_WIDTH = 0.05 * FIELD_SCALE            # 围栏板条宽度 (m)
FENCE_CLEARANCE = 0.10 * FIELD_SCALE        # 围栏内侧与场地边缘的净空 (m)

# 世界底板（场地以外区域）
WORLD_FLOOR_SIZE = 24.0 * FIELD_SCALE       # 足够大的白色底板边长 (m)
WORLD_FLOOR_THICKNESS = 0.02 * FIELD_SCALE  # 厚度 (m)
# 世界底板上表面位于 z=0，主场地地板直接放置其上
WORLD_FLOOR_CENTER_Z = -WORLD_FLOOR_THICKNESS / 2.0

# 货箱（透明塑料盒，五壁无盖）
BOX_LENGTH = 0.28 * FIELD_SCALE             # 外尺寸：长 280mm × scale
BOX_WIDTH = 0.20 * FIELD_SCALE              # 外尺寸：宽 200mm × scale
BOX_HEIGHT = 0.12 * FIELD_SCALE             # 外尺寸：高 120mm × scale
BOX_INNER_LENGTH = 0.25 * FIELD_SCALE       # 内尺寸：长 250mm × scale
BOX_INNER_WIDTH = 0.17 * FIELD_SCALE        # 内尺寸：宽 170mm × scale
BOX_INNER_HEIGHT = 0.12 * FIELD_SCALE       # 内尺寸：高 120mm × scale
BOX_WALL = (BOX_LENGTH - BOX_INNER_LENGTH) / 2.0  # 侧壁厚 15mm
BOX_BOTTOM_THICKNESS = 0.005 * FIELD_SCALE  # 可视化底板厚度

# A4 纸堆叠置物台
A4_LENGTH = 0.30 * FIELD_SCALE              # x (m)
A4_WIDTH = 0.21 * FIELD_SCALE               # y (m)
A4_HEIGHT = 0.05 * FIELD_SCALE              # 单层高度 (m)
A4_LAYERS = 3                 # 每个取货位层数 → 总高 0.15 m

# 圆柱障碍物
OBSTACLE_DIAMETER = 0.102 * FIELD_SCALE     # 直径 (m)
OBSTACLE_RADIUS = OBSTACLE_DIAMETER / 2.0
OBSTACLE_HEIGHT = 0.50 * FIELD_SCALE        # 高度 (m)

# 数字标签
LABEL_SIZE = 0.11 * FIELD_SCALE             # 标签边长 (m)
LABEL_THICKNESS = 0.004 * FIELD_SCALE       # 白色标签底牌厚度 (m)
LABEL_FACE_OFFSET = 0.010 * FIELD_SCALE     # 标签离货箱外壁的距离
LABEL_DIGIT_GAP = 0.002 * FIELD_SCALE       # 数字条段与白色底牌之间的间隙

# 地面标识
CENTER_LINE_THICKNESS = 0.005 * FIELD_SCALE   # 中心线厚度 (m)
CENTER_LINE_HEIGHT = 0.001 * FIELD_SCALE      # 中心线高度（略高于地面防 z-fighting）
MARK_HEIGHT = 0.001 * FIELD_SCALE             # 地面标识高度 (m)

# 简化机器人（用于路径规划仿真，后续直接强制修改 Robot 根节点世界坐标）
ROBOT_LENGTH = 0.36             # x 方向车身长度 (m)
ROBOT_WIDTH = 0.30              # y 方向车身宽度 (m)
ROBOT_HEIGHT = 0.16             # z 方向车身高度 (m)
ROBOT_START_XY = (0.0, 0.0)     # 初始出生在场地中心
ROBOT_BODY_CENTER_Z = FIELD_TOP_Z + ROBOT_HEIGHT / 2.0

# 机器人传感器挂载
SENSOR_VISUALS_ENABLED = False
LIDAR_RADIUS = 0.045
LIDAR_HEIGHT = 0.035
LIDAR_CENTER_Z = FIELD_TOP_Z + ROBOT_HEIGHT + LIDAR_HEIGHT / 2.0
RGB_CAMERA_SIZE = (0.055, 0.035, 0.035)
RGB_CAMERA_CENTER = (
    ROBOT_LENGTH / 2.0 + RGB_CAMERA_SIZE[0] / 2.0,
    0.0,
    FIELD_TOP_Z + ROBOT_HEIGHT * 0.70,
)

# 起始区：距左墙 1000mm、位于中线 y=0。用白色线框表示，避免实心蓝色圆盘干扰观察。
START_AREA_CENTER = (-1.0 * FIELD_SCALE, 0.0)   # 左内墙 x=-4.0，+2.0m → x=-2.0
START_AREA_RADIUS = 0.20 * FIELD_SCALE          # 半径 200mm × scale

# 中心区域：场地正中 400mm×400mm 标识框
CENTRAL_AREA_SIZE = 0.40 * FIELD_SCALE          # 边长 400mm × scale

# 取货位：靠左侧取货区，三货位沿 y 排列，货箱均竖向放置
# 格式：(名称, (x, y), 朝向)，朝向 vertical=长边沿 y，horizontal=长边沿 x
PICKUP_SLOTS = [
    ("pickup_1", (-1.75 * FIELD_SCALE, 0.6 * FIELD_SCALE), "vertical"),
    ("pickup_2", (-1.75 * FIELD_SCALE, 0.0), "vertical"),
    ("pickup_3", (-1.75 * FIELD_SCALE, -0.6 * FIELD_SCALE), "vertical"),
]
PICKUP_BOX_BOTTOM_Z = FIELD_TOP_Z + A4_LAYERS * A4_HEIGHT  # 0.1535 m

# 放置位：编号 4~8 自上而下；4/8 横向放置，5/6/7 竖向放置
# 格式：(名称, (x, y), 标签编号, 朝向)
PLACE_SLOTS = [
    ("place_4", (1.76 * FIELD_SCALE, 0.8 * FIELD_SCALE), 4, "horizontal"),
    ("place_5", (1.8 * FIELD_SCALE, 0.4 * FIELD_SCALE), 5, "vertical"),
    ("place_6", (1.8 * FIELD_SCALE, 0.0), 6, "vertical"),
    ("place_7", (1.8 * FIELD_SCALE, -0.4 * FIELD_SCALE), 7, "vertical"),
    ("place_8", (1.76 * FIELD_SCALE, -0.8 * FIELD_SCALE), 8, "horizontal"),
]
PLACE_BOX_BOTTOM_Z = FIELD_TOP_Z

# 圆柱障碍物：位于中线 y=0，距左墙 1200mm / 2800mm → x=-0.8 / 0.8
OBSTACLE_POSITIONS = [
    ("obstacle_1", (-0.8 * FIELD_SCALE, 0.0)),
    ("obstacle_2", (0.8 * FIELD_SCALE, 0.0)),
]

# 取货区 / 放置区参考竖线 (名称, x, y_min, y_max)
ZONE_MARK_LINES = [
    ("pickup_zone_line", -1.6 * FIELD_SCALE, -0.75 * FIELD_SCALE, 0.75 * FIELD_SCALE),
    ("place_zone_line", 1.8 * FIELD_SCALE, -0.95 * FIELD_SCALE, 0.95 * FIELD_SCALE),
]

# 输出文件
OUTPUT_USD = "logistics_field_stage.usd"
# None 表示每次运行都重新随机；填整数（如 2026）则固定随机顺序，方便复现实验。
RANDOM_SEED = None

# 视觉检测任务：取货区液体颜色与放置区数字标签随机排列
LIQUID_HEIGHT = BOX_HEIGHT - BOX_BOTTOM_THICKNESS - 0.01 * FIELD_SCALE
LIQUID_MARGIN = 0.02 * FIELD_SCALE
LIQUID_COLORS = {
    "yellow": (0.95, 0.78, 0.08),
    "green": (0.05, 0.75, 0.15),
    "white": (0.95, 0.95, 0.90),
}
DIGIT_LABEL_NUMBERS = [1, 2, 3, 4, 5]
DIGIT_STROKE_WIDTH = 0.018 * FIELD_SCALE
DIGIT_STROKE_DEPTH = 0.006 * FIELD_SCALE

# 材质颜色 (RGB 0~1)，参考官方图纸/3D 效果图
COLORS = {
    "floor": (0.18, 0.18, 0.18),       # 深灰色主场地地板
    "world_floor": (0.95, 0.95, 0.94), # 场外底板近白色
    "fence": (0.78, 0.12, 0.12),       # 红色围栏
    "box": (0.88, 0.96, 1.0),          # 近无色透明塑料货箱，便于正面观察液体
    "place_box": (0.86, 0.90, 0.92),    # 放置区不透明浅灰塑料盒，便于看清外侧数字标签
    "a4": (0.95, 0.88, 0.62),          # 浅黄 A4 纸堆
    "obstacle": (0.95, 0.78, 0.12),    # 黄色圆柱障碍物
    "label_bg": (1.0, 1.0, 1.0),       # 完全不透明白色标签底
    "label_text": (1.0, 0.02, 0.0),    # 醒目红色数字标签
    "center_line": (0.90, 0.15, 0.15), # 红色中心线
    "start_area": (0.25, 0.55, 0.95),  # 亮蓝起始区
    "zone_mark": (0.90, 0.90, 0.90),   # 浅灰区标线
    "robot_body": (0.12, 0.18, 0.28),  # 深蓝灰机器人底盘
    "lidar": (0.02, 0.02, 0.02),        # 黑色激光雷达
    "camera": (0.05, 0.05, 0.05),       # 深灰 RGB 相机外壳
    "digit_text": (1.0, 0.02, 0.0),      # 醒目红色数字标签
}

OPACITIES = {
    "box": 0.16,
    "start_area": 0.40,
}


# ---------------------------------------------------------------------------
# 布局辅助
# ---------------------------------------------------------------------------

def box_size_for_orientation(orientation: str) -> tuple[float, float, float]:
    """
    根据货箱朝向返回外尺寸 (x, y, z)。

    vertical:   长边 280mm 沿 y 轴（放置区默认）
    horizontal: 长边 280mm 沿 x 轴（取货区中间位）
    """
    if orientation == "horizontal":
        return (BOX_LENGTH, BOX_WIDTH, BOX_HEIGHT)
    return (BOX_WIDTH, BOX_LENGTH, BOX_HEIGHT)


def a4_footprint_for_orientation(orientation: str) -> tuple[float, float]:
    """A4 置物台在 xy 平面的 (x方向尺寸, y方向尺寸)，朝向与货箱一致。"""
    if orientation == "horizontal":
        return (A4_LENGTH, A4_WIDTH)
    return (A4_WIDTH, A4_LENGTH)


# ---------------------------------------------------------------------------
# 材质与几何体创建函数
# ---------------------------------------------------------------------------

def create_material(
    stage: Usd.Stage,
    path: str,
    color: tuple[float, float, float],
    opacity: float = 1.0,
    roughness: float = 0.5,
    metallic: float = 0.0,
    emissive_color: tuple[float, float, float] | None = None,
) -> UsdShade.Material:
    """
    创建 UsdPreviewSurface 材质并返回 Material prim。

    roughness: 0=镜面光滑，1=完全漫反射粗糙
    metallic:  0=非金属，1=金属
    """
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if emissive_color is not None:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive_color))
    if opacity < 1.0:
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _bind_material(prim: Usd.Prim, material: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def make_fixed_static_collider(prim: Usd.Prim) -> None:
    """把 prim 设置为固定静态碰撞体，作为不会被物理推动的地面使用。"""
    UsdPhysics.CollisionAPI.Apply(prim)
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body.CreateRigidBodyEnabledAttr(False)


def _set_xform(
    xformable: UsdGeom.Xformable,
    position: tuple[float, float, float],
    scale: tuple[float, float, float] | None = None,
) -> None:
    """设置平移；可选非均匀缩放（配合 size=1 的 Cube）。"""
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*position))
    if scale is not None:
        xformable.AddScaleOp().Set(Gf.Vec3d(*scale))


def create_cuboid(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    position: tuple[float, float, float],
    scale: tuple[float, float, float],
    material: UsdShade.Material,
) -> Usd.Prim:
    """
    创建立方体（Cuboid）。

    position: 几何中心世界坐标 (x, y, z)
    scale:    实际尺寸 (长, 宽, 高)，内部使用 size=1 的 Cube 非均匀缩放
    """
    path = f"{parent_path}/{name}"
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    _set_xform(UsdGeom.Xformable(cube), position, scale)
    _bind_material(cube.GetPrim(), material)
    return cube.GetPrim()


def create_cylinder(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    position: tuple[float, float, float],
    radius: float,
    height: float,
    material: UsdShade.Material,
) -> Usd.Prim:
    """
    创建 z 轴竖直圆柱。

    position: 圆柱几何中心 (x, y, z)；底面 z = position_z - height/2
    """
    path = f"{parent_path}/{name}"
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetAxisAttr().Set(UsdGeom.Tokens.z)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    _set_xform(UsdGeom.Xformable(cyl), position)
    _bind_material(cyl.GetPrim(), material)
    return cyl.GetPrim()


def create_open_box(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    center_xy: tuple[float, float],
    bottom_z: float,
    size: tuple[float, float, float],
    wall_thickness: float,
    material: UsdShade.Material,
) -> Usd.Prim:
    """
    创建无盖透明塑料货箱（5 个薄壁 Cuboid：底 + 前后左右）。

    center_xy: 货箱底面中心在 xy 平面上的坐标
    bottom_z:  货箱底面 z 高度
    size:      外尺寸 (length_x, width_y, height_z)
    """
    lx, wy, hz = size
    cx, cy = center_xy
    wt = wall_thickness
    bottom_t = BOX_BOTTOM_THICKNESS

    # 底板使用较薄厚度；侧壁保持 120mm 标称高度，使外形高度符合货箱规格。
    floor_z = bottom_z + bottom_t / 2.0
    wall_h = hz
    wall_z = bottom_z + hz / 2.0

    group_path = f"{parent_path}/{name}"
    UsdGeom.Scope.Define(stage, group_path)

    parts = [
        # 底板
        (f"{name}_bottom", (cx, cy, floor_z), (lx, wy, bottom_t)),
        # 前壁 (+y)
        (f"{name}_front", (cx, cy + wy / 2.0 - wt / 2.0, wall_z), (lx, wt, wall_h)),
        # 后壁 (-y)
        (f"{name}_back", (cx, cy - wy / 2.0 + wt / 2.0, wall_z), (lx, wt, wall_h)),
        # 左壁 (-x)
        (f"{name}_left", (cx - lx / 2.0 + wt / 2.0, cy, wall_z), (wt, wy - 2.0 * wt, wall_h)),
        # 右壁 (+x)
        (f"{name}_right", (cx + lx / 2.0 - wt / 2.0, cy, wall_z), (wt, wy - 2.0 * wt, wall_h)),
    ]

    for part_name, pos, scl in parts:
        create_cuboid(stage, group_path, part_name, pos, scl, material)

    return stage.GetPrimAtPath(group_path)


def create_liquid_fill(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    center_xy: tuple[float, float],
    bottom_z: float,
    box_size: tuple[float, float, float],
    material: UsdShade.Material,
    color_name: str,
) -> Usd.Prim:
    """在透明货箱内部创建一块有颜色的液体体积。"""
    lx, wy, _ = box_size
    cx, cy = center_xy
    liquid_x = max(lx - 2.0 * (BOX_WALL + LIQUID_MARGIN), 0.02)
    liquid_y = max(wy - 2.0 * (BOX_WALL + LIQUID_MARGIN), 0.02)
    liquid_z = bottom_z + BOX_BOTTOM_THICKNESS + LIQUID_HEIGHT / 2.0
    liquid = create_cuboid(
        stage,
        parent_path,
        name,
        (cx, cy, liquid_z),
        (liquid_x, liquid_y, LIQUID_HEIGHT),
        material,
    )
    liquid.CreateAttribute("vision:liquidColor", Sdf.ValueTypeNames.String).Set(color_name)
    return liquid


def create_a4_stack(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    center_xy: tuple[float, float],
    num_layers: int,
    material: UsdShade.Material,
    orientation: str = "vertical",
) -> Usd.Prim:
    """
    创建 A4 纸堆叠置物台。

    center_xy: 堆叠底面中心 xy
    num_layers: 层数，每层 A4_HEIGHT 高
    orientation: 与上方货箱朝向一致，horizontal 时 A4 长边沿 x
    """
    cx, cy = center_xy
    foot_x, foot_y = a4_footprint_for_orientation(orientation)
    group_path = f"{parent_path}/{name}"
    UsdGeom.Scope.Define(stage, group_path)

    for i in range(num_layers):
        # 第 i 层（0-based）中心 z
        z = FIELD_TOP_Z + A4_HEIGHT / 2.0 + i * A4_HEIGHT
        layer_name = f"{name}_layer_{i + 1}"
        create_cuboid(
            stage,
            group_path,
            layer_name,
            (cx, cy, z),
            (foot_x, foot_y, A4_HEIGHT),
            material,
        )

    return stage.GetPrimAtPath(group_path)


def create_flat_disc(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    center_xy: tuple[float, float],
    radius: float,
    material: UsdShade.Material,
) -> Usd.Prim:
    """创建贴地圆盘（用于起始区等圆形标识）。"""
    cx, cy = center_xy
    cyl = UsdGeom.Cylinder.Define(stage, f"{parent_path}/{name}")
    cyl.GetAxisAttr().Set(UsdGeom.Tokens.z)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(MARK_HEIGHT)
    _set_xform(UsdGeom.Xformable(cyl), (cx, cy, FIELD_TOP_Z + MARK_HEIGHT / 2.0))
    _bind_material(cyl.GetPrim(), material)
    return cyl.GetPrim()


def create_rect_frame(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    center_xy: tuple[float, float],
    size: float,
    line_width: float,
    height: float,
    material: UsdShade.Material,
) -> Usd.Prim:
    """创建贴地正方形边框，避免用实心面片遮挡下方地板。"""
    cx, cy = center_xy
    half_size = size / 2.0
    z = FIELD_TOP_Z + height / 2.0

    group_path = f"{parent_path}/{name}"
    UsdGeom.Scope.Define(stage, group_path)

    edges = [
        (f"{name}_north", (cx, cy + half_size, z), (size, line_width, height)),
        (f"{name}_south", (cx, cy - half_size, z), (size, line_width, height)),
        (f"{name}_east", (cx + half_size, cy, z), (line_width, size, height)),
        (f"{name}_west", (cx - half_size, cy, z), (line_width, size, height)),
    ]

    for edge_name, pos, scl in edges:
        create_cuboid(stage, group_path, edge_name, pos, scl, material)

    return stage.GetPrimAtPath(group_path)


def create_label(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    box_center: tuple[float, float, float],
    number: int,
    box_size: tuple[float, float, float],
    bg_material: UsdShade.Material,
    text_material: UsdShade.Material,
) -> Usd.Prim:
    """
    在货箱长侧面（朝向 -x，面向场地中心）贴白色编号标签。

    标签 prim 命名为 label_{number}，便于后续替换贴图。
    数字暂以 prim 名称标注，不依赖 Text prim。
    """
    bx, by, bz = box_center
    bl, bw, bh = box_size

    # 标签贴在朝向场地中心（-x）的侧面外侧，并略微外凸，避免被透明箱体和阴影遮挡。
    label_z = bz
    label_x = bx - bl / 2.0 - LABEL_FACE_OFFSET - LABEL_THICKNESS / 2.0

    label_name = f"label_{number}"
    label_path = f"{parent_path}/{label_name}"
    label = create_cuboid(
        stage,
        parent_path,
        label_name,
        (label_x, by, label_z),
        (LABEL_THICKNESS, LABEL_SIZE, LABEL_SIZE),
        bg_material,
    )

    # 在 prim 上写入自定义属性，方便脚本读取编号
    label.CreateAttribute("logistics:labelNumber", Sdf.ValueTypeNames.Int).Set(number)
    create_digit_on_label(
        stage,
        parent_path,
        f"{label_name}_digit",
        number,
        (
            label_x
            - LABEL_THICKNESS / 2.0
            - LABEL_DIGIT_GAP
            - DIGIT_STROKE_DEPTH / 2.0,
            by,
            label_z,
        ),
        text_material,
    )
    return label


def create_digit_on_label(
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    number: int,
    center: tuple[float, float, float],
    material: UsdShade.Material,
) -> Usd.Prim:
    """用七段数码管样式的红色凸起长方体在标签上绘制 1-5 数字。"""
    digit_root = f"{parent_path}/{name}"
    UsdGeom.Scope.Define(stage, digit_root)

    segments = digit_segments(number)
    cx, cy, cz = center
    digit_w = LABEL_SIZE * 0.55
    digit_h = LABEL_SIZE * 0.78
    half_w = digit_w / 2.0
    half_h = digit_h / 2.0

    segment_specs = {
        "a": ((cx, cy, cz + half_h), (DIGIT_STROKE_DEPTH, digit_w, DIGIT_STROKE_WIDTH)),
        "b": ((cx, cy - half_w, cz + half_h / 2.0), (DIGIT_STROKE_DEPTH, DIGIT_STROKE_WIDTH, digit_h / 2.0)),
        "c": ((cx, cy - half_w, cz - half_h / 2.0), (DIGIT_STROKE_DEPTH, DIGIT_STROKE_WIDTH, digit_h / 2.0)),
        "d": ((cx, cy, cz - half_h), (DIGIT_STROKE_DEPTH, digit_w, DIGIT_STROKE_WIDTH)),
        "e": ((cx, cy + half_w, cz - half_h / 2.0), (DIGIT_STROKE_DEPTH, DIGIT_STROKE_WIDTH, digit_h / 2.0)),
        "f": ((cx, cy + half_w, cz + half_h / 2.0), (DIGIT_STROKE_DEPTH, DIGIT_STROKE_WIDTH, digit_h / 2.0)),
        "g": ((cx, cy, cz), (DIGIT_STROKE_DEPTH, digit_w, DIGIT_STROKE_WIDTH)),
    }

    for segment_name in segments:
        pos, scale = segment_specs[segment_name]
        create_cuboid(stage, digit_root, f"{name}_{segment_name}", pos, scale, material)

    return stage.GetPrimAtPath(digit_root)


def digit_segments(number: int) -> tuple[str, ...]:
    """返回七段数码管中需要点亮的段。"""
    segment_map = {
        1: ("b", "c"),
        2: ("a", "b", "g", "e", "d"),
        3: ("a", "b", "g", "c", "d"),
        4: ("f", "g", "b", "c"),
        5: ("a", "f", "g", "c", "d"),
    }
    return segment_map[number]


def create_robot_with_sensors(
    stage: Usd.Stage,
    root: str,
    body_material: UsdShade.Material,
    lidar_material: UsdShade.Material,
    camera_material: UsdShade.Material,
) -> Usd.Prim:
    """
    创建用于路径规划仿真的机器人根节点和传感器。

    Robot 根节点的平移就是机器人在世界坐标系中的位置；后续移动命令可直接
    修改 /World/LogisticsField/Robot 的 translate，实现强制位姿更新。
    真实机器人视觉模型在 Isaac Sim 打开 stage 后由 final_robot_model.py 导入。
    """
    robot_path = f"{root}/Robot"
    robot = UsdGeom.Xform.Define(stage, robot_path)
    robot_xf = UsdGeom.Xformable(robot)
    robot_xf.ClearXformOpOrder()
    robot_xf.AddTranslateOp().Set(Gf.Vec3d(ROBOT_START_XY[0], ROBOT_START_XY[1], 0.0))

    sensors_root = f"{robot_path}/Sensors"
    UsdGeom.Scope.Define(stage, sensors_root)

    # 顶部 2D 激光雷达：当前先作为传感器挂载点与参数描述，后续可接入真实 Isaac LiDAR。
    lidar_path = f"{sensors_root}/Lidar"
    lidar = UsdGeom.Xform.Define(stage, lidar_path)
    lidar_xf = UsdGeom.Xformable(lidar)
    lidar_xf.ClearXformOpOrder()
    lidar_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LIDAR_CENTER_Z))
    lidar_prim = lidar.GetPrim()
    lidar_prim.CreateAttribute("sensor:type", Sdf.ValueTypeNames.String).Set("planar_lidar")
    lidar_prim.CreateAttribute("sensor:frame", Sdf.ValueTypeNames.String).Set("lidar")
    lidar_prim.CreateAttribute("sensor:minRange", Sdf.ValueTypeNames.Float).Set(0.05)
    lidar_prim.CreateAttribute("sensor:maxRange", Sdf.ValueTypeNames.Float).Set(6.0)
    lidar_prim.CreateAttribute("sensor:horizontalFovDeg", Sdf.ValueTypeNames.Float).Set(360.0)
    lidar_prim.CreateAttribute("sensor:horizontalResolution", Sdf.ValueTypeNames.Int).Set(720)
    if SENSOR_VISUALS_ENABLED:
        create_cylinder(
            stage,
            lidar_path,
            "LidarBody",
            (0.0, 0.0, 0.0),
            LIDAR_RADIUS,
            LIDAR_HEIGHT,
            lidar_material,
        )

        # 前向 RGB 相机：USD Camera 可被 Isaac/viewport 识别，视觉外壳用于场景中定位。
        create_cuboid(
            stage,
            sensors_root,
            "RGBCameraBody",
            RGB_CAMERA_CENTER,
            RGB_CAMERA_SIZE,
            camera_material,
        )
        camera = UsdGeom.Camera.Define(stage, f"{sensors_root}/RGBCamera")
        camera_xf = UsdGeom.Xformable(camera)
        camera_xf.ClearXformOpOrder()
        camera_xf.AddTranslateOp().Set(Gf.Vec3d(*RGB_CAMERA_CENTER))
        camera_xf.AddRotateYOp().Set(-90.0)  # USD 相机默认看向 -Z，旋转后看向机器人 +X 前方。
        camera.GetFocalLengthAttr().Set(18.0)
        camera.GetHorizontalApertureAttr().Set(20.955)
        camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.02, 20.0))
        camera.GetPrim().CreateAttribute("sensor:type", Sdf.ValueTypeNames.String).Set("rgb_camera")
        camera.GetPrim().CreateAttribute("sensor:frame", Sdf.ValueTypeNames.String).Set("rgb_camera")

    return robot.GetPrim()


# ---------------------------------------------------------------------------
# 场地组装
# ---------------------------------------------------------------------------

def create_field_floor(
    stage: Usd.Stage,
    root: str,
    material: UsdShade.Material,
) -> None:
    """4m × 2m × 3.5mm 主场地地板，底面贴世界地面并固定。"""
    floor = create_cuboid(
        stage,
        root,
        "Floor",
        (0.0, 0.0, FIELD_CENTER_Z),
        (FIELD_LENGTH, FIELD_WIDTH, FIELD_THICKNESS),
        material,
    )
    make_fixed_static_collider(floor)


def create_fences(
    stage: Usd.Stage,
    root: str,
    material: UsdShade.Material,
) -> None:
    """
    围绕场地四边的低矮围栏，底面在 z = 0。

    围栏内侧距场地边缘留有 FENCE_CLEARANCE 净空，使围栏整体略大于场地。
    四根角柱使南北围栏首尾相接，形成封闭框。
    """
    half_l = FIELD_LENGTH / 2.0
    half_w = FIELD_WIDTH / 2.0
    fence_z = FIELD_TOP_Z + FENCE_HEIGHT / 2.0  # 底面贴主场地上表面

    # 围栏内侧到场地边缘的净空 + 围栏宽度的一半 = 围栏中心线到场地边缘的距离
    offset_l = half_l + FENCE_CLEARANCE + FENCE_WIDTH / 2.0  # 东西围栏中心 x
    offset_w = half_w + FENCE_CLEARANCE + FENCE_WIDTH / 2.0  # 南北围栏中心 y

    # 南北围栏长度 = 东西两条围栏外侧之间的距离
    ns_span = 2.0 * offset_l + FENCE_WIDTH  # 含角部完全闭合

    fences = [
        # 北 (+y)
        ("Fence_North", (0.0,  offset_w, fence_z), (ns_span, FENCE_WIDTH, FENCE_HEIGHT)),
        # 南 (-y)
        ("Fence_South", (0.0, -offset_w, fence_z), (ns_span, FENCE_WIDTH, FENCE_HEIGHT)),
        # 东 (+x)
        ("Fence_East",  ( offset_l, 0.0, fence_z), (FENCE_WIDTH, FIELD_WIDTH + 2.0 * FENCE_CLEARANCE, FENCE_HEIGHT)),
        # 西 (-x)
        ("Fence_West",  (-offset_l, 0.0, fence_z), (FENCE_WIDTH, FIELD_WIDTH + 2.0 * FENCE_CLEARANCE, FENCE_HEIGHT)),
    ]

    scope = UsdGeom.Scope.Define(stage, f"{root}/Fences")
    for fname, pos, scl in fences:
        create_cuboid(stage, scope.GetPath().pathString, fname, pos, scl, material)


def create_ground_marks(
    stage: Usd.Stage,
    root: str,
    center_line_mat: UsdShade.Material,
    start_area_mat: UsdShade.Material,
    zone_mark_mat: UsdShade.Material,
) -> None:
    """中心十字线、起始区线框、中心方框、取货/放置区参考线。"""
    marks_root = f"{root}/GroundMarks"
    UsdGeom.Scope.Define(stage, marks_root)

    # 竖向中心线：x = 0，沿 y 方向
    create_cuboid(
        stage,
        marks_root,
        "CenterLine_Vertical",
        (0.0, 0.0, FIELD_TOP_Z + CENTER_LINE_HEIGHT / 2.0),
        (CENTER_LINE_THICKNESS, FIELD_WIDTH, CENTER_LINE_HEIGHT),
        center_line_mat,
    )

    # 横向中心线：y = 0，沿 x 方向（图纸虚线中轴）
    create_cuboid(
        stage,
        marks_root,
        "CenterLine_Horizontal",
        (0.0, 0.0, FIELD_TOP_Z + CENTER_LINE_HEIGHT / 2.0),
        (FIELD_LENGTH, CENTER_LINE_THICKNESS, CENTER_LINE_HEIGHT),
        center_line_mat,
    )

    # 中心区域：400mm × 400mm 方框。用边框而不是实心薄片，避免中央地面看起来被透明/挖空。
    create_rect_frame(
        stage,
        marks_root,
        "CentralArea",
        (0.0, 0.0),
        CENTRAL_AREA_SIZE,
        CENTER_LINE_THICKNESS,
        MARK_HEIGHT,
        zone_mark_mat,
    )

    # 起始区：用 400mm × 400mm 白色线框表示，不再使用实心蓝色圆盘。
    create_rect_frame(
        stage,
        marks_root,
        "StartArea",
        START_AREA_CENTER,
        START_AREA_RADIUS * 2.0,
        CENTER_LINE_THICKNESS,
        MARK_HEIGHT,
        zone_mark_mat,
    )

    # 取货区 / 放置区竖向参考线
    for mark_name, x_pos, y_min, y_max in ZONE_MARK_LINES:
        y_center = (y_min + y_max) / 2.0
        y_span = y_max - y_min
        create_cuboid(
            stage,
            marks_root,
            mark_name,
            (x_pos, y_center, FIELD_TOP_Z + MARK_HEIGHT / 2.0),
            (CENTER_LINE_THICKNESS, y_span, MARK_HEIGHT),
            zone_mark_mat,
        )


def create_lighting_and_camera(stage: Usd.Stage) -> None:
    """在世界根节点添加平行光与默认视角相机（不在 LogisticsField 子树内）。"""
    # 平行光
    light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
    light.CreateIntensityAttr(2500.0)
    light.CreateAngleAttr(1.0)
    light_xf = UsdGeom.Xformable(light)
    light_xf.ClearXformOpOrder()
    light_xf.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 45.0, 0.0))

    # 环境 dome 光（柔和补光）
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(800.0)

    # 相机：从场地斜上方俯视
    cam = UsdGeom.Camera.Define(stage, "/World/MainCamera")
    cam_xf = UsdGeom.Xformable(cam)
    cam_xf.ClearXformOpOrder()
    cam_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, -4.5, 3.2))
    cam_xf.AddRotateXYZOp().Set(Gf.Vec3f(52.0, 0.0, 0.0))
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))


def build_logistics_field(stage: Usd.Stage, root_path: str = "/World/LogisticsField") -> None:
    """组装完整物流比赛场地。"""
    rng = random.Random(RANDOM_SEED) if RANDOM_SEED is not None else random.Random()

    # 确保 /World 存在
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")

    UsdGeom.Scope.Define(stage, root_path)

    # --- 材质 ---
    mats = {
        # 浅灰木质地板：不透明、高粗糙度（亚光）
        "floor": create_material(
            stage, f"{root_path}/Materials/LightGrayWoodFloorMat",
            COLORS["floor"], roughness=0.82, metallic=0.0,
        ),
        # 场外白色底板：不透明哑光
        "world_floor": create_material(
            stage, f"{root_path}/Materials/WorldFloorMat",
            COLORS["world_floor"], opacity=1.0, roughness=0.9, metallic=0.0,
        ),
        "fence": create_material(stage, f"{root_path}/Materials/FenceMat", COLORS["fence"]),
        "box": create_material(
            stage,
            f"{root_path}/Materials/BoxMat",
            COLORS["box"],
            OPACITIES["box"],
            roughness=0.08,
        ),
        "place_box": create_material(
            stage,
            f"{root_path}/Materials/PlaceBoxMat",
            COLORS["place_box"],
            opacity=1.0,
            roughness=0.45,
            metallic=0.0,
        ),
        "a4": create_material(stage, f"{root_path}/Materials/A4Mat", COLORS["a4"]),
        "obstacle": create_material(stage, f"{root_path}/Materials/ObstacleMat", COLORS["obstacle"]),
        "label_bg": create_material(
            stage,
            f"{root_path}/Materials/LabelBgMat",
            COLORS["label_bg"],
            opacity=1.0,
            roughness=0.55,
            metallic=0.0,
        ),
        "center_line": create_material(
            stage, f"{root_path}/Materials/CenterLineMat", COLORS["center_line"]
        ),
        "start_area": create_material(
            stage,
            f"{root_path}/Materials/StartAreaMat",
            COLORS["start_area"],
            OPACITIES["start_area"],
        ),
        "zone_mark": create_material(stage, f"{root_path}/Materials/ZoneMarkMat", COLORS["zone_mark"]),
        "robot_body": create_material(
            stage, f"{root_path}/Materials/RobotBodyMat", COLORS["robot_body"], roughness=0.65,
        ),
        "lidar": create_material(
            stage, f"{root_path}/Materials/LidarMat", COLORS["lidar"], roughness=0.45,
        ),
        "camera": create_material(
            stage, f"{root_path}/Materials/RGBCameraMat", COLORS["camera"], roughness=0.45,
        ),
        "digit_text": create_material(
            stage,
            f"{root_path}/Materials/DigitTextMat",
            COLORS["digit_text"],
            opacity=1.0,
            roughness=0.35,
            metallic=0.0,
            emissive_color=(0.35, 0.0, 0.0),
        ),
    }
    for liquid_name, liquid_color in LIQUID_COLORS.items():
        mats[f"liquid_{liquid_name}"] = create_material(
            stage,
            f"{root_path}/Materials/Liquid_{liquid_name.capitalize()}Mat",
            liquid_color,
            roughness=0.35,
            emissive_color=tuple(channel * 0.35 for channel in liquid_color),
        )

    # --- 基础场地 ---
    # 世界底板（场地以外白色区域，上表面为 z=0，主场地地板直接放置其上）
    create_cuboid(
        stage,
        root_path,
        "WorldFloor",
        (0.0, 0.0, WORLD_FLOOR_CENTER_Z),
        (WORLD_FLOOR_SIZE, WORLD_FLOOR_SIZE, WORLD_FLOOR_THICKNESS),
        mats["world_floor"],
    )
    # 场内浅灰木质地板（4m×2m×3.5mm，底面 z=0，固定静态）
    create_field_floor(stage, root_path, mats["floor"])
    create_fences(stage, root_path, mats["fence"])
    create_ground_marks(
        stage,
        root_path,
        mats["center_line"],
        mats["start_area"],
        mats["zone_mark"],
    )

    # --- 取货区：A4 置物台 + 空货箱 ---
    pickup_root = f"{root_path}/PickupZone"
    UsdGeom.Scope.Define(stage, pickup_root)

    pickup_liquids = list(LIQUID_COLORS)
    rng.shuffle(pickup_liquids)

    for (slot_name, (px, py), orientation), liquid_name in zip(PICKUP_SLOTS, pickup_liquids):
        box_size = box_size_for_orientation(orientation)
        create_a4_stack(
            stage,
            pickup_root,
            f"{slot_name}_stack",
            (px, py),
            A4_LAYERS,
            mats["a4"],
            orientation,
        )
        create_open_box(
            stage,
            pickup_root,
            f"{slot_name}_box",
            (px, py),
            PICKUP_BOX_BOTTOM_Z,
            box_size,
            BOX_WALL,
            mats["box"],
        )
        create_liquid_fill(
            stage,
            pickup_root,
            f"{slot_name}_liquid_{liquid_name}",
            (px, py),
            PICKUP_BOX_BOTTOM_Z,
            box_size,
            mats[f"liquid_{liquid_name}"],
            liquid_name,
        )

    # --- 放置区：地面货箱 + 编号标签（按图纸朝向摆放）---
    place_root = f"{root_path}/PlaceZone"
    UsdGeom.Scope.Define(stage, place_root)

    place_labels = DIGIT_LABEL_NUMBERS[:]
    rng.shuffle(place_labels)

    for (slot_name, (px, py), _, orientation), label_num in zip(PLACE_SLOTS, place_labels):
        box_size = box_size_for_orientation(orientation)
        box_center_z = PLACE_BOX_BOTTOM_Z + BOX_HEIGHT / 2.0
        box_center = (px, py, box_center_z)
        create_open_box(
            stage,
            place_root,
            f"{slot_name}_box",
            (px, py),
            PLACE_BOX_BOTTOM_Z,
            box_size,
            BOX_WALL,
            mats["place_box"],
        )
        create_label(
            stage,
            place_root,
            slot_name,
            box_center,
            label_num,
            box_size,
            mats["label_bg"],
            mats["digit_text"],
        )

    # --- 中部圆柱障碍物 ---
    obstacle_root = f"{root_path}/Obstacles"
    UsdGeom.Scope.Define(stage, obstacle_root)
    for obs_name, (ox, oy) in OBSTACLE_POSITIONS:
        # 底面贴主场地上表面
        create_cylinder(
            stage,
            obstacle_root,
            obs_name,
            (ox, oy, FIELD_TOP_Z + OBSTACLE_HEIGHT / 2.0),
            OBSTACLE_RADIUS,
            OBSTACLE_HEIGHT,
            mats["obstacle"],
        )

    # --- 简化机器人与传感器 ---
    create_robot_with_sensors(
        stage,
        root_path,
        mats["robot_body"],
        mats["lidar"],
        mats["camera"],
    )

    # --- 光照与相机 ---
    create_lighting_and_camera(stage)


def save_stage(stage: Usd.Stage, output_path: str) -> str:
    """保存 USD 到指定路径，返回绝对路径。"""
    abs_path = os.path.abspath(output_path)
    stage.GetRootLayer().Export(abs_path)
    return abs_path


def create_new_stage(output_path: str) -> Usd.Stage:
    """创建新的干净 USD Stage 并设置米制单位。"""
    abs_path = os.path.abspath(output_path)
    stage = Usd.Stage.CreateNew(abs_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    return stage


def main(output_usd: str = OUTPUT_USD) -> str:
    """主入口：建场 → 保存 → 返回文件路径。"""
    stage = create_new_stage(output_usd)
    build_logistics_field(stage)
    saved = save_stage(stage, output_usd)
    print(f"[LogisticsField] 场景已保存: {saved}")
    return saved


# ---------------------------------------------------------------------------
# 运行方式
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # standalone：若不在 Isaac Sim 内，先启动 SimulationApp
    _sim_app = None
    _in_isaac = "omni.kit.app" in sys.modules or "omni.usd" in sys.modules

    if not _in_isaac:
        try:
            from isaacsim import SimulationApp

            _sim_app = SimulationApp({"headless": True})
            _in_isaac = True
        except ImportError:
            pass

    saved_path = main()

    if _sim_app is not None:
        # standalone Python：保持应用运行，便于在 GUI 中查看
        while _sim_app.is_running():
            _sim_app.update()
    elif _in_isaac:
        # Script Editor：尝试打开刚保存的场景
        try:
            import omni.usd

            ctx = omni.usd.get_context()
            ctx.open_stage(saved_path)
            print("[LogisticsField] 已在当前 Isaac Sim 会话中打开场景。")
        except Exception:
            pass
    else:
        print(
            "[LogisticsField] 已生成 USD 文件。\n"
            f"  路径: {saved_path}\n"
            "  请使用 Isaac Sim 自带 Python 运行本脚本，或将 USD 拖入 Isaac Sim 查看。"
        )
