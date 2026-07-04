#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
物流比赛场地路径规划演示脚本。

功能：
  1. 生成并打开当前 logistics_field_stage.usd；
  2. 从机器人/雷达当前位姿出发；
  3. 依次到达 3 个取货箱前，相机朝向箱子并停留 5s；
  4. 按最短路径依次到达 5 个放置箱前，相机朝向箱子并停留 5s；
  5. 回到启动时读取到的初始位姿。

说明：
  - 当前阶段使用 USD prim 位姿作为“雷达估计位姿”的来源：
    机器人顶部 /World/LogisticsField/Robot/Sensors/Lidar 的世界坐标即为雷达位置。
  - 运动控制采用强制更新 Robot 根节点位姿，便于课程设计早期验证路径与视角；
    后续接入真实 LiDAR/控制器时，可替换 get_lidar_pose() 与 set_robot_pose()。
"""

from __future__ import annotations

import heapq
import math
import os
import time
from dataclasses import dataclass
from typing import Iterable

from isaacsim import SimulationApp

# 必须先启动 SimulationApp，再导入 pxr / omni 相关模块。
simulation_app = SimulationApp({"headless": os.environ.get("ISAAC_HEADLESS") == "1"})

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from create_logistics_field import (
    FIELD_LENGTH,
    FIELD_TOP_Z,
    FIELD_WIDTH,
    OBSTACLE_POSITIONS,
    OBSTACLE_RADIUS,
    OUTPUT_USD,
    PICKUP_SLOTS,
    PLACE_SLOTS,
    ROBOT_LENGTH,
    main as build_scene,
)
from final_robot_model import FINAL_MODEL_PATH, import_final_robot_model


ROOT_PATH = "/World/LogisticsField"
ROBOT_PATH = f"{ROOT_PATH}/Robot"
LIDAR_PATH = f"{ROBOT_PATH}/Sensors/Lidar"
PICKUP_ZONE_PATH = f"{ROOT_PATH}/PickupZone"
PLACE_ZONE_PATH = f"{ROOT_PATH}/PlaceZone"
INDICATOR_PATH = f"{ROBOT_PATH}/VisionIndicators"
INDICATOR_VISUALS_ENABLED = False

# 运动参数
LINEAR_SPEED = 0.35          # m/s，演示用平移速度
ANGULAR_SPEED = 0.75         # rad/s，限制转向速度，避免瞬间闪转
UPDATE_DT = 1.0 / 60.0       # s，控制更新周期
DWELL_SECONDS = 5.0          # 每个箱子前停留时间

# A* 网格参数
GRID_RESOLUTION = 0.05       # m
BOUNDARY_MARGIN = 0.18       # 机器人中心离场地边界的最小距离
OBSTACLE_CLEARANCE = 0.28    # 圆柱障碍物膨胀半径，含机器人半宽和安全余量

# 箱子前方拍照距离：机器人中心到箱子中心的 x 向距离。
# Final 真实模型比原方块更长，0.46m 会让模型前缘与放大后的箱体干涉。
PICKUP_BOX_VIEW_DISTANCE = 0.80
PLACE_BOX_VIEW_DISTANCE = 0.70
PICKUP_STAND_LATERAL_OFFSET = -0.10
PICKUP_STAND_EXTRA_LATERAL_OFFSETS = {
    "pickup_1": -0.05,
}
PICKUP_STAND_EXTRA_VIEW_DISTANCES = {
    "pickup_1": -0.10,
}

# 路径后处理参数：用可视线合并 A* 网格折线，并在运动时减少微小航向抖动。
LINE_OF_SIGHT_STEP = GRID_RESOLUTION * 0.5
WAYPOINT_REACHED_DISTANCE = 0.035

# 视觉结果指示灯：白色液体用黄色灯替代，便于观察。
INDICATOR_COLORS = {
    "red": (1.0, 0.0, 0.0),
    "green": (0.0, 1.0, 0.0),
    "yellow": (1.0, 0.85, 0.0),
    "off": (0.03, 0.03, 0.03),
}
LIQUID_TO_LAMP = {
    "yellow": "yellow",
    "green": "green",
    "white": "yellow",
}
DIGIT_TO_LAMP = {
    1: "green",
    2: "red",
    3: "yellow",
    4: None,
    5: None,
}

# 取货机构动作参数。URDF 中 f-b / l-r 是水平直线关节，claw-lift 为 Z 向直线关节，claw-roll 为 Z 轴旋转关节。
FB_JOINT_MIN = -0.20
FB_JOINT_MAX = 0.10
LR_JOINT_MIN = -0.20
LR_JOINT_MAX = 0.10
CLAW_BOX_INSERT_XY_MARGIN = 0.01
CLAW_BOX_INSERT_Z_MARGIN = 0.03
CLAW_LIQUID_INSERT_DEPTH = 0.04
CLAW_LIFT_UP = 0.0
CLAW_LIFT_DOWN = -0.50
CLAW_ROLL_HOME = -math.pi / 2.0
CLAW_ROLL_DUMP = 0.0
LEFT_CLAW_OPEN = 0.0
LEFT_CLAW_CLOSED = 0.02
RIGHT_CLAW_OPEN = 0.0
RIGHT_CLAW_CLOSED = -0.02
CLAW_SLIDE_AXIS = Gf.Vec3d(-0.96773, -0.25199, 0.0)
CLAW_MOTION_SECONDS = 1.2
PLATFORM_HOME_FB = -0.05
PLATFORM_HOME_LR = -0.05
STORE_LIFT_UP = 0.0
STORE_LIFT_TRAVEL = -0.25
STORE_LIFT_UNLOAD = -0.15
STORE_LIFT_SECONDS = 1.0
GATE_CLOSED = 0.0
GATE_OPEN = 0.04
GATE_MOTION_SECONDS = 0.6
UNLOAD_DWELL_SECONDS = 1.0
BANK_SORT_ANGLES = {
    "green": 0.0,
    "white": 0.70,
    "yellow": 1.32,
}
BANK_MOTION_SECONDS = 0.8
UNLOAD_GATE_BY_LABEL = {
    1: "r_gate",
    2: "l_gate",
    3: "f_gate",
}
UNLOAD_YAW_OFFSET_BY_LABEL = {
    1: math.pi / 2.0,
    2: -math.pi / 2.0,
    3: math.pi,
}
UNLOAD_PLATFORM_POSE_BY_LABEL = {
    1: (0.10, PLATFORM_HOME_LR),
    2: (-0.20, PLATFORM_HOME_LR),
    3: (PLATFORM_HOME_FB, -0.20),
}


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class InspectionTarget:
    name: str
    stand_xy: tuple[float, float]
    look_at_xy: tuple[float, float]


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def step_yaw_towards(current: float, target: float, dt: float) -> float:
    """按 ANGULAR_SPEED 限制，从 current 平滑转向 target。"""
    error = normalize_angle(target - current)
    max_step = ANGULAR_SPEED * dt
    if abs(error) <= max_step:
        return target
    return normalize_angle(current + math.copysign(max_step, error))


def yaw_to_quat(yaw: float) -> Gf.Quatf:
    """绕 z 轴 yaw 角转 USD 四元数。"""
    return Gf.Quatf(math.cos(yaw / 2.0), Gf.Vec3f(0.0, 0.0, math.sin(yaw / 2.0)))


def create_preview_material(
    stage: Usd.Stage,
    path: str,
    color: tuple[float, float, float],
    emissive: bool = False,
) -> UsdShade.Material:
    """创建用于指示灯的简单材质。"""
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.25)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    if emissive:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind_material(prim: Usd.Prim, material: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI(prim).Bind(material)


def create_vision_indicator_lamps(stage: Usd.Stage) -> None:
    """
    在机器人顶部靠后位置创建红/绿/黄三盏小灯。

    顶部中央保留给激光雷达，灯放在 x=-0.11 的一排，避免与雷达重叠。
    """
    UsdGeom.Scope.Define(stage, INDICATOR_PATH)

    materials = {
        "red": create_preview_material(stage, f"{INDICATOR_PATH}/Materials/RedLampMat", INDICATOR_COLORS["red"], True),
        "green": create_preview_material(stage, f"{INDICATOR_PATH}/Materials/GreenLampMat", INDICATOR_COLORS["green"], True),
        "yellow": create_preview_material(stage, f"{INDICATOR_PATH}/Materials/YellowLampMat", INDICATOR_COLORS["yellow"], True),
        "off": create_preview_material(stage, f"{INDICATOR_PATH}/Materials/OffLampMat", INDICATOR_COLORS["off"], False),
    }

    if not INDICATOR_VISUALS_ENABLED:
        return

    lamp_layout = {
        "red": (-0.11, -0.085, FIELD_TOP_Z + 0.19),
        "green": (-0.11, 0.0, FIELD_TOP_Z + 0.19),
        "yellow": (-0.11, 0.085, FIELD_TOP_Z + 0.19),
    }
    for lamp_name, pos in lamp_layout.items():
        sphere = UsdGeom.Sphere.Define(stage, f"{INDICATOR_PATH}/{lamp_name}_lamp")
        sphere.GetRadiusAttr().Set(0.026)
        xform = UsdGeom.Xformable(sphere)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
        bind_material(sphere.GetPrim(), materials["off"])

    # 在 scope 上记录材质路径，方便切换绑定。
    scope = stage.GetPrimAtPath(INDICATOR_PATH)
    for key in materials:
        scope.CreateAttribute(f"vision:{key}Material", Sdf.ValueTypeNames.String).Set(
            f"{INDICATOR_PATH}/Materials/{key.capitalize() if key != 'off' else 'Off'}LampMat"
        )


def set_vision_indicator(stage: Usd.Stage, active_lamp: str | None) -> None:
    """点亮 red/green/yellow 中的一盏；None 表示全灭。"""
    off_mat = UsdShade.Material(stage.GetPrimAtPath(f"{INDICATOR_PATH}/Materials/OffLampMat"))
    active_mat = None
    if active_lamp is not None:
        mat_name = f"{active_lamp.capitalize()}LampMat"
        active_mat = UsdShade.Material(stage.GetPrimAtPath(f"{INDICATOR_PATH}/Materials/{mat_name}"))

    for lamp_name in ("red", "green", "yellow"):
        prim = stage.GetPrimAtPath(f"{INDICATOR_PATH}/{lamp_name}_lamp")
        if not prim:
            continue
        bind_material(prim, active_mat if lamp_name == active_lamp and active_mat else off_mat)


def find_final_model_part(stage: Usd.Stage, part_name: str | tuple[str, ...]) -> Usd.Prim | None:
    """查找 Final URDF 导入后的 link prim。Isaac 会把连字符改成下划线。"""
    part_names = (part_name,) if isinstance(part_name, str) else part_name
    for candidate in part_names:
        direct_path = f"{FINAL_MODEL_PATH}/{candidate}"
        prim = stage.GetPrimAtPath(direct_path)
        if prim:
            return prim

    root = stage.GetPrimAtPath(FINAL_MODEL_PATH)
    if root:
        for child in Usd.PrimRange(root):
            if child.GetName() in part_names:
                return child
    return None


def z_axis_quat(angle: float) -> Gf.Quatd:
    """绕 Z 轴的局部旋转四元数。"""
    return Gf.Quatd(
        math.cos(angle / 2.0),
        Gf.Vec3d(0.0, 0.0, math.sin(angle / 2.0)),
    )


def get_translate_op(xform: UsdGeom.Xformable) -> UsdGeom.XformOp:
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return xform.AddTranslateOp()


def get_orient_op(xform: UsdGeom.Xformable) -> UsdGeom.XformOp:
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            return op
    return xform.AddOrientOp()


@dataclass
class ClawPart:
    translate_op: UsdGeom.XformOp
    orient_op: UsdGeom.XformOp
    base_translate: Gf.Vec3d
    base_orient: Gf.Quatd


class ClawAnimator:
    """用 link 局部变换展示取货机构动作。"""

    def __init__(self, stage: Usd.Stage):
        self.stage = stage
        self.fb_part = self._part_control(("f_b", "f_b_platform"))
        self.lr_part = self._part_control(("l_r", "l_r_platform"))
        self.roll_part = self._part_control("claw_roll")
        self.lift_part = self._part_control("claw_lift")
        self.left_claw_part = self._part_control("left_claw")
        self.right_claw_part = self._part_control("right_claw")
        self.store_lift_parts = tuple(
            part
            for part in (
                self._part_control("store"),
            )
            if part is not None
        )
        self.gate_parts = {
            name: part
            for name, part in (
                ("l_gate", self._part_control("l_gate")),
                ("r_gate", self._part_control("r_gate")),
                ("f_gate", self._part_control("f_gate")),
            )
            if part is not None
        }
        self.bank_parts = tuple(
            part
            for part in (
                self._part_control("l_bank"),
                self._part_control("r_bank"),
            )
            if part is not None
        )
        self.current_pose: tuple[float, float, float, float, float, float] | None = None
        self.current_store_lift = STORE_LIFT_UP
        self.current_bank_angle = 0.0
        self.current_gate_positions = {name: GATE_CLOSED for name in self.gate_parts}

    def _part_control(self, part_name: str | tuple[str, ...]) -> ClawPart | None:
        prim = find_final_model_part(self.stage, part_name)
        if prim is None:
            print(f"[ClawAction] 找不到机构 link: {part_name}", flush=True)
            return None

        xform = UsdGeom.Xformable(prim)
        translate_op = get_translate_op(xform)
        orient_op = get_orient_op(xform)

        base_value = translate_op.Get()
        base = Gf.Vec3d(base_value) if base_value is not None else Gf.Vec3d(0.0, 0.0, 0.0)

        orient_value = orient_op.Get()
        if orient_value is None:
            base_orient = Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
        else:
            base_orient = Gf.Quatd(
                float(orient_value.GetReal()),
                Gf.Vec3d(orient_value.GetImaginary()),
            )

        return ClawPart(translate_op, orient_op, base, base_orient)

    @property
    def available(self) -> bool:
        return all((
            self.fb_part,
            self.lr_part,
            self.roll_part,
            self.lift_part,
            self.left_claw_part,
            self.right_claw_part,
        ))

    @staticmethod
    def rotate_z(vec: Gf.Vec3d, angle: float) -> Gf.Vec3d:
        c = math.cos(angle)
        s = math.sin(angle)
        return Gf.Vec3d(c * vec[0] - s * vec[1], s * vec[0] + c * vec[1], vec[2])

    def set_part_pose(
        self,
        part: ClawPart,
        roll_origin: Gf.Vec3d,
        roll: float,
        offset_before_roll: Gf.Vec3d,
    ) -> None:
        unrolled_translate = part.base_translate + offset_before_roll
        rolled_translate = roll_origin + self.rotate_z(unrolled_translate - roll_origin, roll)
        part.translate_op.Set(rolled_translate)
        part.orient_op.Set(z_axis_quat(roll) * part.base_orient)

    def set_pose(
        self,
        fb: float,
        lr: float,
        lift: float,
        roll: float,
        left_claw: float,
        right_claw: float,
    ) -> None:
        if not self.available:
            return

        fb = min(FB_JOINT_MAX, max(FB_JOINT_MIN, fb))
        lr = min(LR_JOINT_MAX, max(LR_JOINT_MIN, lr))
        platform_offset = Gf.Vec3d(lr, -fb, 0.0)
        fb_offset = Gf.Vec3d(0.0, -fb, 0.0)
        roll_origin = self.roll_part.base_translate + platform_offset
        lift_offset = Gf.Vec3d(0.0, 0.0, lift)
        left_open_offset = CLAW_SLIDE_AXIS * left_claw
        right_open_offset = CLAW_SLIDE_AXIS * right_claw

        # URDF 导入后的 link 在 USD 中不一定形成可传播的父子 Xform 层级。
        # 因此这里显式重算整条机构链：f-b/l-r 影响下游所有 link，roll/lift 继续影响夹爪。
        self.fb_part.translate_op.Set(self.fb_part.base_translate + fb_offset)
        self.fb_part.orient_op.Set(self.fb_part.base_orient)
        self.lr_part.translate_op.Set(self.lr_part.base_translate + platform_offset)
        self.lr_part.orient_op.Set(self.lr_part.base_orient)
        store_lift_offset = Gf.Vec3d(0.0, 0.0, self.current_store_lift)
        for part in self.store_lift_parts:
            part.translate_op.Set(part.base_translate + platform_offset + store_lift_offset)
            part.orient_op.Set(part.base_orient)
        for name, part in self.gate_parts.items():
            gate_offset = Gf.Vec3d(0.0, 0.0, self.current_gate_positions.get(name, GATE_CLOSED))
            part.translate_op.Set(part.base_translate + platform_offset + store_lift_offset + gate_offset)
            part.orient_op.Set(part.base_orient)
        for part in self.bank_parts:
            part.translate_op.Set(part.base_translate + platform_offset + store_lift_offset)
            part.orient_op.Set(z_axis_quat(self.current_bank_angle) * part.base_orient)
        self.set_part_pose(self.roll_part, roll_origin, roll, platform_offset)
        self.set_part_pose(self.lift_part, roll_origin, roll, platform_offset + lift_offset)
        self.set_part_pose(self.left_claw_part, roll_origin, roll, platform_offset + lift_offset + left_open_offset)
        self.set_part_pose(self.right_claw_part, roll_origin, roll, platform_offset + lift_offset + right_open_offset)
        self.current_pose = (fb, lr, lift, roll, left_claw, right_claw)

    def set_bank_angle(self, angle: float) -> None:
        self.current_bank_angle = min(1.32, max(0.0, angle))
        if self.current_pose is None:
            self.set_pose(0.0, 0.0, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN)
        else:
            self.set_pose(*self.current_pose)

    def animate_bank_angle(self, target: float, seconds: float) -> None:
        start = self.current_bank_angle
        steps = max(1, int(seconds / UPDATE_DT))
        for i in range(1, steps + 1):
            if not simulation_app.is_running():
                break
            t = i / steps
            self.set_bank_angle(start + (target - start) * t)
            simulation_app.update()
            time.sleep(UPDATE_DT)

    def set_sorting_bank_for_liquid(self, liquid_color: str | None) -> None:
        if not self.bank_parts:
            return
        if liquid_color not in BANK_SORT_ANGLES:
            print(f"[Sort] 未知豆子颜色 {liquid_color}，bank 保持 {self.current_bank_angle:.2f}", flush=True)
            return
        target = BANK_SORT_ANGLES[liquid_color]
        print(f"[Sort] {liquid_color} -> bank angle {target:.2f}", flush=True)
        self.animate_bank_angle(target, BANK_MOTION_SECONDS)

    def set_store_lift(self, store_lift: float) -> None:
        store_lift = min(0.1, max(-0.3, store_lift))
        self.current_store_lift = store_lift
        if self.current_pose is None:
            self.set_pose(0.0, 0.0, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN)
        else:
            self.set_pose(*self.current_pose)

    def animate_store_lift(self, target: float, seconds: float) -> None:
        start = self.current_store_lift
        steps = max(1, int(seconds / UPDATE_DT))
        for i in range(1, steps + 1):
            if not simulation_app.is_running():
                break
            t = i / steps
            self.set_store_lift(start + (target - start) * t)
            simulation_app.update()
            time.sleep(UPDATE_DT)

    def lower_store_for_travel(self) -> None:
        if not self.store_lift_parts:
            return
        if abs(self.current_store_lift - STORE_LIFT_TRAVEL) < 1e-4:
            return
        print(f"[ClawAction] 小车启动前料仓降低: store-lift={STORE_LIFT_TRAVEL:.2f}", flush=True)
        self.animate_store_lift(STORE_LIFT_TRAVEL, STORE_LIFT_SECONDS)

    def set_gate_position(self, gate_name: str, position: float) -> None:
        if gate_name not in self.gate_parts:
            print(f"[Unload] 找不到料仓门: {gate_name}", flush=True)
            return
        self.current_gate_positions[gate_name] = min(GATE_OPEN, max(GATE_CLOSED, position))
        if self.current_pose is None:
            self.set_pose(0.0, 0.0, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN)
        else:
            self.set_pose(*self.current_pose)

    def animate_gate_position(self, gate_name: str, target: float, seconds: float) -> None:
        start = self.current_gate_positions.get(gate_name, GATE_CLOSED)
        steps = max(1, int(seconds / UPDATE_DT))
        for i in range(1, steps + 1):
            if not simulation_app.is_running():
                break
            t = i / steps
            self.set_gate_position(gate_name, start + (target - start) * t)
            simulation_app.update()
            time.sleep(UPDATE_DT)

    def close_all_gates(self) -> None:
        for gate_name in tuple(self.gate_parts):
            if abs(self.current_gate_positions.get(gate_name, GATE_CLOSED) - GATE_CLOSED) > 1e-4:
                self.animate_gate_position(gate_name, GATE_CLOSED, GATE_MOTION_SECONDS * 0.5)

    def animate_platform_to(self, fb: float, lr: float, seconds: float) -> None:
        if not self.available:
            return
        if self.current_pose is None:
            self.set_pose(PLATFORM_HOME_FB, PLATFORM_HOME_LR, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN)
        assert self.current_pose is not None
        end_pose = (fb, lr, self.current_pose[2], self.current_pose[3], self.current_pose[4], self.current_pose[5])
        if all(abs(a - b) < 1e-4 for a, b in zip(self.current_pose, end_pose)):
            return
        self.animate_pose(self.current_pose, end_pose, seconds)

    def run_unload_sequence(self, label_number: int | None) -> None:
        gate_name = UNLOAD_GATE_BY_LABEL.get(label_number)
        if gate_name is None:
            print(f"[Unload] 标签 {label_number} 不需要卸货动作", flush=True)
            return
        unload_platform_pose = UNLOAD_PLATFORM_POSE_BY_LABEL.get(
            label_number,
            (PLATFORM_HOME_FB, PLATFORM_HOME_LR),
        )
        print(
            f"[Unload] 标签 {label_number}: 云台移动到 f-b={unload_platform_pose[0]:.2f}, "
            f"l-r={unload_platform_pose[1]:.2f} -> 抬升料仓 -> 打开 {gate_name}",
            flush=True,
        )
        self.close_all_gates()
        self.animate_platform_to(*unload_platform_pose, CLAW_MOTION_SECONDS * 0.6)
        self.animate_store_lift(STORE_LIFT_UNLOAD, STORE_LIFT_SECONDS)
        self.animate_gate_position(gate_name, GATE_OPEN, GATE_MOTION_SECONDS)
        end_time = time.time() + UNLOAD_DWELL_SECONDS
        while simulation_app.is_running() and time.time() < end_time:
            simulation_app.update()
            time.sleep(UPDATE_DT)
        self.animate_gate_position(gate_name, GATE_CLOSED, GATE_MOTION_SECONDS)
        self.animate_store_lift(STORE_LIFT_TRAVEL, STORE_LIFT_SECONDS)
        self.animate_platform_to(PLATFORM_HOME_FB, PLATFORM_HOME_LR, CLAW_MOTION_SECONDS * 0.6)

    def reset_platform_for_departure(self) -> None:
        if not self.available or self.current_pose is None:
            return
        home_pose = (PLATFORM_HOME_FB, PLATFORM_HOME_LR, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN)
        if all(abs(a - b) < 1e-4 for a, b in zip(self.current_pose, home_pose)):
            return
        print("[ClawAction] 离开停靠点前平台复位", flush=True)
        self.animate_pose(self.current_pose, home_pose, CLAW_MOTION_SECONDS)

    def animate_pose(
        self,
        start: tuple[float, float, float, float, float, float],
        end: tuple[float, float, float, float, float, float],
        seconds: float,
    ) -> tuple[float, float, float, float, float, float]:
        steps = max(1, int(seconds / UPDATE_DT))
        for i in range(1, steps + 1):
            if not simulation_app.is_running():
                break
            t = i / steps
            pose = tuple(s + (e - s) * t for s, e in zip(start, end))
            self.set_pose(*pose)
            simulation_app.update()
            time.sleep(UPDATE_DT)
        return end

    def _bbox_range_local(self, prim: Usd.Prim) -> tuple[Gf.Vec3d, Gf.Vec3d] | None:
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bound = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        model_prim = self.stage.GetPrimAtPath(FINAL_MODEL_PATH)
        if bound.IsEmpty() or not model_prim:
            return None

        world_to_model = UsdGeom.Xformable(model_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
        bmin = bound.GetMin()
        bmax = bound.GetMax()
        corners = [
            world_to_model.Transform(Gf.Vec3d(x, y, z))
            for x in (bmin[0], bmax[0])
            for y in (bmin[1], bmax[1])
            for z in (bmin[2], bmax[2])
        ]
        local_min = Gf.Vec3d(
            min(c[0] for c in corners),
            min(c[1] for c in corners),
            min(c[2] for c in corners),
        )
        local_max = Gf.Vec3d(
            max(c[0] for c in corners),
            max(c[1] for c in corners),
            max(c[2] for c in corners),
        )
        return local_min, local_max

    @staticmethod
    def _entry_axis_target(current: float, low: float, high: float) -> float:
        if current < low:
            return low + CLAW_BOX_INSERT_XY_MARGIN
        if current > high:
            return high - CLAW_BOX_INSERT_XY_MARGIN
        return (low + high) / 2.0

    def _claw_lower_center_local(self) -> Gf.Vec3d | None:
        if not (self.left_claw_part and self.right_claw_part):
            return None

        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        left_prim = find_final_model_part(self.stage, "left_claw")
        right_prim = find_final_model_part(self.stage, "right_claw")
        model_prim = self.stage.GetPrimAtPath(FINAL_MODEL_PATH)
        if left_prim is None or right_prim is None or not model_prim:
            return None

        left_bound = bbox_cache.ComputeWorldBound(left_prim).ComputeAlignedBox()
        right_bound = bbox_cache.ComputeWorldBound(right_prim).ComputeAlignedBox()
        if left_bound.IsEmpty() or right_bound.IsEmpty():
            return None

        lower_world = Gf.Vec3d(
            (left_bound.GetMidpoint()[0] + right_bound.GetMidpoint()[0]) / 2.0,
            (left_bound.GetMidpoint()[1] + right_bound.GetMidpoint()[1]) / 2.0,
            min(left_bound.GetMin()[2], right_bound.GetMin()[2]),
        )
        world_to_model = UsdGeom.Xformable(model_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
        return world_to_model.Transform(lower_world)

    def compute_pickup_alignment(self, pickup_name: str) -> tuple[float, float]:
        if self.current_pose is None:
            current_fb = 0.0
            current_lr = 0.0
        else:
            current_fb, current_lr = self.current_pose[0], self.current_pose[1]

        box_prim = self.stage.GetPrimAtPath(f"{PICKUP_ZONE_PATH}/{pickup_name}_box")
        if not box_prim:
            print(f"[ClawAction] 找不到取货箱: {pickup_name}_box，水平滑台保持当前位置。", flush=True)
            return current_fb, current_lr

        box_range = self._bbox_range_local(box_prim)
        claw_center = self._claw_lower_center_local()
        if box_range is None or claw_center is None:
            print("[ClawAction] 无法计算夹爪/货箱包围盒，水平滑台保持当前位置。", flush=True)
            return current_fb, current_lr

        box_min, box_max = box_range
        target_x = self._entry_axis_target(float(claw_center[0]), float(box_min[0]), float(box_max[0]))
        target_y = self._entry_axis_target(float(claw_center[1]), float(box_min[1]), float(box_max[1]))
        delta_x = target_x - float(claw_center[0])
        delta_y = target_y - float(claw_center[1])
        target_lr = min(LR_JOINT_MAX, max(LR_JOINT_MIN, current_lr + delta_x))
        target_fb = min(FB_JOINT_MAX, max(FB_JOINT_MIN, current_fb - delta_y))
        print(
            "[ClawAction] 水平对准 "
            f"{pickup_name}: claw=({claw_center[0]:.3f}, {claw_center[1]:.3f}), "
            f"target=({target_x:.3f}, {target_y:.3f}), "
            f"fb={target_fb:.3f}, lr={target_lr:.3f}",
            flush=True,
        )
        return target_fb, target_lr

    def compute_pickup_down_lift(self, pickup_name: str) -> float:
        current_lift = self.current_pose[2] if self.current_pose is not None else CLAW_LIFT_UP
        box_prim = self.stage.GetPrimAtPath(f"{PICKUP_ZONE_PATH}/{pickup_name}_box")
        if not box_prim:
            return CLAW_LIFT_DOWN
        liquid_prim = self._pickup_liquid_prim(pickup_name)

        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        box_bound = bbox_cache.ComputeWorldBound(box_prim).ComputeAlignedBox()
        claw_center = self._claw_lower_center_local()
        model_prim = self.stage.GetPrimAtPath(FINAL_MODEL_PATH)
        if box_bound.IsEmpty() or claw_center is None or not model_prim:
            return CLAW_LIFT_DOWN

        world_to_model = UsdGeom.Xformable(model_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
        box_bottom = world_to_model.Transform(
            Gf.Vec3d(box_bound.GetMidpoint()[0], box_bound.GetMidpoint()[1], box_bound.GetMin()[2])
        )
        target_z = float(box_bottom[2] + CLAW_BOX_INSERT_Z_MARGIN)
        liquid_top_z = None
        if liquid_prim:
            liquid_bound = bbox_cache.ComputeWorldBound(liquid_prim).ComputeAlignedBox()
            if not liquid_bound.IsEmpty():
                liquid_top = world_to_model.Transform(
                    Gf.Vec3d(liquid_bound.GetMidpoint()[0], liquid_bound.GetMidpoint()[1], liquid_bound.GetMax()[2])
                )
                liquid_top_z = float(liquid_top[2])
                target_z = max(target_z, liquid_top_z - CLAW_LIQUID_INSERT_DEPTH)

        target_lift = current_lift + float(target_z - claw_center[2])
        target_lift = min(CLAW_LIFT_UP, target_lift)
        liquid_top_text = f"{liquid_top_z:.3f}" if liquid_top_z is not None else "n/a"
        print(
            "[ClawAction] 下降高度 "
            f"{pickup_name}: claw_z={claw_center[2]:.3f}, "
            f"box_bottom_z={box_bottom[2]:.3f}, "
            f"liquid_top_z={liquid_top_text}, "
            f"lift={target_lift:.3f}",
            flush=True,
        )
        return target_lift

    def _pickup_liquid_prim(self, pickup_name: str) -> Usd.Prim | None:
        pickup_zone = self.stage.GetPrimAtPath(PICKUP_ZONE_PATH)
        if not pickup_zone:
            return None
        prefix = f"{pickup_name}_liquid_"
        for child in pickup_zone.GetChildren():
            if child.GetName().startswith(prefix):
                return child
        return None

    def run_pickup_sequence(self, pickup_name: str) -> None:
        if not self.available:
            print("[ClawAction] 取货机构不完整，跳过动作。", flush=True)
            return

        print("[ClawAction] 取货区动作: 水平对准 -> 下降 -> 闭合 -> 上升 -> claw-roll转到0 -> 张开", flush=True)
        pickup_ready_pose = (0.0, 0.0, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN)
        if self.current_pose is None:
            pose = pickup_ready_pose
            self.set_pose(*pose)
            simulation_app.update()
        else:
            pose = self.animate_pose(
                self.current_pose,
                pickup_ready_pose,
                CLAW_MOTION_SECONDS,
            )

        target_fb, target_lr = self.compute_pickup_alignment(pickup_name)
        pose = self.animate_pose(
            pose,
            (target_fb, target_lr, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN),
            CLAW_MOTION_SECONDS,
        )
        down_lift = self.compute_pickup_down_lift(pickup_name)
        pose = self.animate_pose(
            pose,
            (target_fb, target_lr, down_lift, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN),
            CLAW_MOTION_SECONDS,
        )
        pose = self.animate_pose(
            pose,
            (target_fb, target_lr, down_lift, CLAW_ROLL_HOME, LEFT_CLAW_CLOSED, RIGHT_CLAW_CLOSED),
            CLAW_MOTION_SECONDS * 0.5,
        )
        pose = self.animate_pose(
            pose,
            (target_fb, target_lr, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_CLOSED, RIGHT_CLAW_CLOSED),
            CLAW_MOTION_SECONDS,
        )
        pose = self.animate_pose(
            pose,
            (target_fb, target_lr, CLAW_LIFT_UP, CLAW_ROLL_DUMP, LEFT_CLAW_CLOSED, RIGHT_CLAW_CLOSED),
            CLAW_MOTION_SECONDS,
        )
        pose = self.animate_pose(
            pose,
            (target_fb, target_lr, CLAW_LIFT_UP, CLAW_ROLL_DUMP, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN),
            CLAW_MOTION_SECONDS * 0.5,
        )
        self.current_pose = pose


def read_pickup_liquid_color(stage: Usd.Stage, pickup_name: str) -> str | None:
    """读取取货区对应箱子的液体颜色属性。"""
    pickup_zone = stage.GetPrimAtPath(PICKUP_ZONE_PATH)
    if not pickup_zone:
        return None

    prefix = f"{pickup_name}_liquid_"
    for child in pickup_zone.GetChildren():
        if not child.GetName().startswith(prefix):
            continue
        attr = child.GetAttribute("vision:liquidColor")
        if attr:
            return attr.Get()
    return None


def read_nearest_place_label_number(
    stage: Usd.Stage,
    box_center_xy: tuple[float, float],
) -> int | None:
    """
    模拟 RGB 相机识别放置区数字。

    当前场景中数字标签已经写入 logistics:labelNumber 属性；这里选取离当前
    look_at 点最近的标签 prim，等价于“相机正对该箱子后读到的数字”。
    后续接真实图像识别时，可替换此函数。
    """
    place_zone = stage.GetPrimAtPath(PLACE_ZONE_PATH)
    if not place_zone:
        return None

    cache = UsdGeom.XformCache()
    best_distance = float("inf")
    best_number: int | None = None

    for child in place_zone.GetChildren():
        attr = child.GetAttribute("logistics:labelNumber")
        if not attr:
            continue
        number = attr.Get()
        if number is None:
            continue

        label_pos = cache.GetLocalToWorldTransform(child).ExtractTranslation()
        distance = math.hypot(
            float(label_pos[0]) - box_center_xy[0],
            float(label_pos[1]) - box_center_xy[1],
        )
        if distance < best_distance:
            best_distance = distance
            best_number = int(number)

    return best_number


def ensure_robot_xform_ops(stage: Usd.Stage):
    """确保 Robot 根 prim 使用 translate + orient 两个 xform op，返回二者。"""
    robot = stage.GetPrimAtPath(ROBOT_PATH)
    if not robot:
        raise RuntimeError(f"找不到机器人 prim: {ROBOT_PATH}")

    xform = UsdGeom.Xformable(robot)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    orient_op = xform.AddOrientOp()
    return translate_op, orient_op


def get_lidar_pose(stage: Usd.Stage, robot_yaw: float) -> Pose2D:
    """
    读取雷达世界位置，作为当前机器人位姿估计。

    简化场景中雷达固定在机器人顶部，因此 x/y 可直接代表机器人平面位置。
    yaw 由当前控制器维护；后续若接入真实传感器，可在这里替换成定位输出。
    """
    lidar = stage.GetPrimAtPath(LIDAR_PATH)
    if not lidar:
        raise RuntimeError(f"找不到雷达 prim: {LIDAR_PATH}")

    world_tf = UsdGeom.XformCache().GetLocalToWorldTransform(lidar)
    p = world_tf.ExtractTranslation()
    return Pose2D(float(p[0]), float(p[1]), robot_yaw)


def set_robot_pose(
    translate_op: UsdGeom.XformOp,
    orient_op: UsdGeom.XformOp,
    pose: Pose2D,
) -> None:
    """强制设置 Robot 根节点世界位姿。"""
    translate_op.Set(Gf.Vec3d(pose.x, pose.y, 0.0))
    orient_op.Set(yaw_to_quat(pose.yaw))


def face_yaw(from_xy: tuple[float, float], to_xy: tuple[float, float]) -> float:
    """计算机器人 +X 前方朝向目标点所需 yaw。"""
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    return math.atan2(dy, dx)


def build_inspection_targets() -> list[InspectionTarget]:
    """根据场景参数生成取货区与放置区的拍照停靠点。"""
    targets: list[InspectionTarget] = []

    # 取货区在左侧，机器人站在箱子靠场地中心一侧，相机朝 -X。
    for slot_name, (box_x, box_y), _orientation in PICKUP_SLOTS:
        lateral_offset = PICKUP_STAND_LATERAL_OFFSET + PICKUP_STAND_EXTRA_LATERAL_OFFSETS.get(slot_name, 0.0)
        view_distance = PICKUP_BOX_VIEW_DISTANCE + PICKUP_STAND_EXTRA_VIEW_DISTANCES.get(slot_name, 0.0)
        stand_xy = (box_x + view_distance, box_y + lateral_offset)
        targets.append(InspectionTarget(slot_name, stand_xy, (box_x, box_y)))

    # 放置区在右侧，机器人站在箱子靠场地中心一侧，相机朝 +X。
    for slot_name, (box_x, box_y), _label_num, _orientation in PLACE_SLOTS:
        stand_xy = (box_x - PLACE_BOX_VIEW_DISTANCE, box_y)
        targets.append(InspectionTarget(slot_name, stand_xy, (box_x, box_y)))

    return targets


def in_field(x: float, y: float) -> bool:
    """判断机器人中心是否位于可通行场地范围内。"""
    half_l = FIELD_LENGTH / 2.0 - BOUNDARY_MARGIN
    half_w = FIELD_WIDTH / 2.0 - BOUNDARY_MARGIN
    return -half_l <= x <= half_l and -half_w <= y <= half_w


def occupied_by_obstacle(x: float, y: float) -> bool:
    """判断点是否落入膨胀后的圆柱障碍物区域。"""
    for _name, (ox, oy) in OBSTACLE_POSITIONS:
        if math.hypot(x - ox, y - oy) <= OBSTACLE_RADIUS + OBSTACLE_CLEARANCE:
            return True
    return False


def is_free(x: float, y: float) -> bool:
    """A* 网格可通行性检查。"""
    return in_field(x, y) and not occupied_by_obstacle(x, y)


def grid_key(point: tuple[float, float]) -> tuple[int, int]:
    return (
        round(point[0] / GRID_RESOLUTION),
        round(point[1] / GRID_RESOLUTION),
    )


def grid_xy(key: tuple[int, int]) -> tuple[float, float]:
    return (key[0] * GRID_RESOLUTION, key[1] * GRID_RESOLUTION)


def nearest_free_key(point: tuple[float, float]) -> tuple[int, int]:
    """如果目标点正好落在膨胀障碍区边缘，则找最近可通行网格。"""
    start = grid_key(point)
    if is_free(*grid_xy(start)):
        return start

    for radius in range(1, 16):
        candidates = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) != radius and abs(dy) != radius:
                    continue
                key = (start[0] + dx, start[1] + dy)
                xy = grid_xy(key)
                if is_free(*xy):
                    candidates.append((math.hypot(xy[0] - point[0], xy[1] - point[1]), key))
        if candidates:
            candidates.sort(key=lambda item: item[0])
            return candidates[0][1]
    raise RuntimeError(f"找不到靠近 {point} 的可通行网格")


def neighbors(key: tuple[int, int]) -> Iterable[tuple[tuple[int, int], float]]:
    """8 邻域网格。"""
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nk = (key[0] + dx, key[1] + dy)
            xy = grid_xy(nk)
            if is_free(*xy):
                yield nk, math.hypot(dx, dy) * GRID_RESOLUTION


def astar_path(start_xy: tuple[float, float], goal_xy: tuple[float, float]) -> list[tuple[float, float]]:
    """在场地平面上规划一段最短路径。"""
    start = nearest_free_key(start_xy)
    goal = nearest_free_key(goal_xy)

    open_heap: list[tuple[float, tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, start))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost_so_far: dict[tuple[int, int], float] = {start: 0.0}

    while open_heap:
        _priority, current = heapq.heappop(open_heap)
        if current == goal:
            break

        for nk, step_cost in neighbors(current):
            new_cost = cost_so_far[current] + step_cost
            if nk not in cost_so_far or new_cost < cost_so_far[nk]:
                cost_so_far[nk] = new_cost
                gx, gy = grid_xy(goal)
                nx, ny = grid_xy(nk)
                heuristic = math.hypot(gx - nx, gy - ny)
                heapq.heappush(open_heap, (new_cost + heuristic, nk))
                came_from[nk] = current

    if goal not in cost_so_far:
        raise RuntimeError(f"A* 规划失败: {start_xy} -> {goal_xy}")

    path_keys = [goal]
    while path_keys[-1] != start:
        path_keys.append(came_from[path_keys[-1]])
    path_keys.reverse()
    return [grid_xy(k) for k in path_keys]


def simplify_path(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """保留方向变化点，减少运动抖动。"""
    if len(path) <= 2:
        return path

    simplified = [path[0]]
    prev_dir: tuple[int, int] | None = None
    for i in range(1, len(path)):
        dx = round((path[i][0] - path[i - 1][0]) / GRID_RESOLUTION)
        dy = round((path[i][1] - path[i - 1][1]) / GRID_RESOLUTION)
        cur_dir = (dx, dy)
        if prev_dir is not None and cur_dir != prev_dir:
            simplified.append(path[i - 1])
        prev_dir = cur_dir
    simplified.append(path[-1])
    return simplified


def has_line_of_sight(
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
) -> bool:
    """检查两点之间是否能用一段直线安全连接。"""
    dx = goal_xy[0] - start_xy[0]
    dy = goal_xy[1] - start_xy[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return True

    steps = max(1, math.ceil(distance / LINE_OF_SIGHT_STEP))
    for i in range(steps + 1):
        t = i / steps
        x = start_xy[0] + dx * t
        y = start_xy[1] + dy * t
        if not is_free(x, y):
            return False
    return True


def shortcut_path(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """
    用可视线压缩 A* 网格路径。

    A* 为了绕障碍会产生大量 45 度短折线；如果两点间无遮挡，直接保留一条
    长直线，机器人移动时航向变化更少。
    """
    if len(path) <= 2:
        return path

    shortcut = [path[0]]
    anchor_index = 0
    while anchor_index < len(path) - 1:
        next_index = len(path) - 1
        while next_index > anchor_index + 1:
            if has_line_of_sight(path[anchor_index], path[next_index]):
                break
            next_index -= 1
        shortcut.append(path[next_index])
        anchor_index = next_index
    return shortcut


def postprocess_path(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """先去掉方向重复点，再用可视线合并成更平滑的直线段。"""
    return shortcut_path(simplify_path(path))


def move_along_path(
    stage: Usd.Stage,
    translate_op: UsdGeom.XformOp,
    orient_op: UsdGeom.XformOp,
    current_pose: Pose2D,
    path: list[tuple[float, float]],
) -> Pose2D:
    """沿路径逐段移动机器人。"""
    pose = current_pose
    if len(path) < 2:
        return pose

    for waypoint in path[1:]:
        while simulation_app.is_running():
            dx = waypoint[0] - pose.x
            dy = waypoint[1] - pose.y
            dist = math.hypot(dx, dy)
            if dist < WAYPOINT_REACHED_DISTANCE:
                pose = Pose2D(waypoint[0], waypoint[1], pose.yaw)
                set_robot_pose(translate_op, orient_op, pose)
                simulation_app.update()
                break

            step = min(LINEAR_SPEED * UPDATE_DT, dist)
            target_yaw = math.atan2(dy, dx)
            yaw = step_yaw_towards(pose.yaw, target_yaw, UPDATE_DT)

            # 麦轮/全向底盘可以沿规划路径平移，同时车身朝向平滑跟随路径方向。
            # 这里平移方向必须使用“当前位置 -> 路径点”的方向，而不是受限后的 yaw；
            # 否则机器人会在转向滞后时走弧线，甚至围着路径点绕圈。
            pose = Pose2D(
                pose.x + dx / dist * step,
                pose.y + dy / dist * step,
                yaw,
            )
            set_robot_pose(translate_op, orient_op, pose)
            simulation_app.update()

    return pose


def rotate_to_face_and_wait(
    translate_op: UsdGeom.XformOp,
    orient_op: UsdGeom.XformOp,
    pose: Pose2D,
    look_at_xy: tuple[float, float],
    seconds: float,
    on_aligned=None,
) -> Pose2D:
    """将相机朝向目标箱子并停留。"""
    target_yaw = face_yaw((pose.x, pose.y), look_at_xy)

    # 先以受限角速度原地平滑转向，避免相机一闪就对准。
    while simulation_app.is_running():
        next_yaw = step_yaw_towards(pose.yaw, target_yaw, UPDATE_DT)
        pose = Pose2D(pose.x, pose.y, next_yaw)
        set_robot_pose(translate_op, orient_op, pose)
        simulation_app.update()
        time.sleep(UPDATE_DT)
        if abs(normalize_angle(target_yaw - pose.yaw)) < 0.01:
            break

    if on_aligned is not None:
        on_aligned()

    end_time = time.time() + seconds
    while simulation_app.is_running() and time.time() < end_time:
        simulation_app.update()
        time.sleep(UPDATE_DT)
    return pose


def rotate_to_yaw(
    translate_op: UsdGeom.XformOp,
    orient_op: UsdGeom.XformOp,
    pose: Pose2D,
    target_yaw: float,
) -> Pose2D:
    while simulation_app.is_running():
        next_yaw = step_yaw_towards(pose.yaw, target_yaw, UPDATE_DT)
        pose = Pose2D(pose.x, pose.y, next_yaw)
        set_robot_pose(translate_op, orient_op, pose)
        simulation_app.update()
        time.sleep(UPDATE_DT)
        if abs(normalize_angle(target_yaw - pose.yaw)) < 0.01:
            break
    return pose


def execute_inspection_route(stage: Usd.Stage) -> None:
    initial_pose = get_lidar_pose(stage, robot_yaw=0.0)
    translate_op, orient_op = ensure_robot_xform_ops(stage)
    set_robot_pose(translate_op, orient_op, initial_pose)
    create_vision_indicator_lamps(stage)
    set_vision_indicator(stage, None)
    claw_animator = ClawAnimator(stage)
    claw_animator.set_pose(0.0, 0.0, CLAW_LIFT_UP, CLAW_ROLL_HOME, LEFT_CLAW_OPEN, RIGHT_CLAW_OPEN)
    simulation_app.update()
    claw_animator.lower_store_for_travel()
    current_pose = initial_pose

    targets = build_inspection_targets()
    print("[PathPlanning] 巡检顺序:")
    for target in targets:
        print(f"  - {target.name}: stand={target.stand_xy}, look_at={target.look_at_xy}")

    for target in targets:
        print(f"[PathPlanning] 前往 {target.name}")
        # 每段规划前读取雷达挂载点世界坐标，作为当前位姿估计。
        current_pose = get_lidar_pose(stage, current_pose.yaw)
        raw_path = astar_path((current_pose.x, current_pose.y), target.stand_xy)
        path = postprocess_path(raw_path)
        current_pose = move_along_path(stage, translate_op, orient_op, current_pose, path)
        place_label_number: int | None = None

        def on_camera_aligned(target_name=target.name):
            nonlocal place_label_number
            if target_name.startswith("pickup_"):
                liquid_color = read_pickup_liquid_color(stage, target_name)
                lamp_color = LIQUID_TO_LAMP.get(liquid_color or "")
                set_vision_indicator(stage, lamp_color)
                print(
                    f"[Vision] {target_name} detected liquid: {liquid_color} "
                    f"-> lamp: {lamp_color}"
                )
                claw_animator.set_sorting_bank_for_liquid(liquid_color)
                claw_animator.run_pickup_sequence(target_name)
                return

            if target_name.startswith("place_"):
                label_number = read_nearest_place_label_number(stage, target.look_at_xy)
                place_label_number = label_number
                lamp_color = DIGIT_TO_LAMP.get(label_number)
                set_vision_indicator(stage, lamp_color)
                print(
                    f"[Vision] {target_name} detected digit: {label_number} "
                    f"-> lamp: {lamp_color or 'off'}"
                )
                return

            set_vision_indicator(stage, None)

        current_pose = rotate_to_face_and_wait(
            translate_op,
            orient_op,
            current_pose,
            target.look_at_xy,
            DWELL_SECONDS,
            on_aligned=on_camera_aligned,
        )
        if target.name.startswith("place_") and place_label_number in UNLOAD_GATE_BY_LABEL:
            unload_yaw = face_yaw(
                (current_pose.x, current_pose.y),
                target.look_at_xy,
            ) + UNLOAD_YAW_OFFSET_BY_LABEL[place_label_number]
            current_pose = rotate_to_yaw(translate_op, orient_op, current_pose, unload_yaw)
            claw_animator.run_unload_sequence(place_label_number)
        elif target.name.startswith("place_"):
            claw_animator.run_unload_sequence(place_label_number)
        claw_animator.reset_platform_for_departure()

    print("[PathPlanning] 返回初始位置")
    raw_path = astar_path((current_pose.x, current_pose.y), (initial_pose.x, initial_pose.y))
    path = postprocess_path(raw_path)
    current_pose = move_along_path(stage, translate_op, orient_op, current_pose, path)
    current_pose = rotate_to_face_and_wait(
        translate_op,
        orient_op,
        current_pose,
        (initial_pose.x + math.cos(initial_pose.yaw), initial_pose.y + math.sin(initial_pose.yaw)),
        1.0,
    )
    print("[PathPlanning] 巡检完成，机器人已回到初始位置。")


def main() -> None:
    saved_path = build_scene(OUTPUT_USD)
    ctx = omni.usd.get_context()
    ctx.open_stage(saved_path)

    # 等待 stage 完成加载。
    for _ in range(30):
        simulation_app.update()

    stage = ctx.get_stage()
    if stage is None:
        raise RuntimeError("USD Stage 加载失败")

    import_final_robot_model(stage)
    execute_inspection_route(stage)

    # 巡检结束后保持窗口打开，便于检查最终位置。
    while simulation_app.is_running():
        simulation_app.update()


if __name__ == "__main__":
    main()
