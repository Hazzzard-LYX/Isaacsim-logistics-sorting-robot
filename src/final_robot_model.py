#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Import the SolidWorks-exported Final robot model into the logistics scene."""

from __future__ import annotations

import math
import shutil
from pathlib import Path

import omni.kit.commands
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT_ROOT
SOURCE_URDF = PROJECT_ROOT / "assets" / "robot" / "final-urdf" / "urdf" / "final-urdf.urdf"
SOURCE_MESH_DIR = PROJECT_ROOT / "assets" / "robot" / "final-urdf" / "meshes"
SOURCE_PACKAGE_NAMES = ("最后的最后的最后", "Final", "final-urdf")
RUNTIME_DIR = WORKSPACE / ".final_urdf_runtime"
RUNTIME_MESH_DIR = RUNTIME_DIR / "meshes"
RUNTIME_URDF = RUNTIME_DIR / "final_robot.urdf"

ROBOT_ROOT_PATH = "/World/LogisticsField/Robot"
FINAL_MODEL_PATH = f"{ROBOT_ROOT_PATH}/FinalModel"

# The Final STL coordinates are exported in meters. X/Y keep the imported model
# centered around Robot root; Z is adjusted dynamically from the mesh bbox.
FINAL_MODEL_REAR_SHIFT = 0.4
FINAL_MODEL_OFFSET = Gf.Vec3d(0.1465 - FINAL_MODEL_REAR_SHIFT, -0.25, 0.0)
FINAL_MODEL_YAW_OFFSET = -math.radians(12.0)
FINAL_MODEL_GROUND_Z = 0.008
DEFAULT_CLAW_ROLL_HOME = -math.pi / 2.0
RED_SORTING_PARTS = ("l_bank", "r_bank", "l_gate", "r_gate", "f_gate")
SORTING_RED_COLOR = (0.92, 0.04, 0.02)


def prepare_runtime_urdf() -> Path:
    """Create a runtime URDF with absolute mesh paths for Isaac's importer."""
    if not SOURCE_URDF.exists():
        raise FileNotFoundError(f"找不到 Final URDF: {SOURCE_URDF}")
    if not SOURCE_MESH_DIR.exists():
        raise FileNotFoundError(f"找不到 Final mesh 目录: {SOURCE_MESH_DIR}")

    text = SOURCE_URDF.read_text(encoding="utf-8")
    RUNTIME_MESH_DIR.mkdir(parents=True, exist_ok=True)
    for old_mesh in RUNTIME_MESH_DIR.glob("*.STL"):
        old_mesh.unlink()
    for mesh in SOURCE_MESH_DIR.glob("*.STL"):
        safe_name = mesh.name.replace("-", "_").replace(" ", "_")
        safe_mesh = RUNTIME_MESH_DIR / safe_name
        shutil.copy2(mesh, safe_mesh)
        for package_name in SOURCE_PACKAGE_NAMES:
            text = text.replace(
                f"package://{package_name}/meshes/{mesh.name}",
                safe_mesh.resolve().as_posix(),
            )

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    RUNTIME_URDF.write_text(text, encoding="utf-8")
    return RUNTIME_URDF


def _remove_prim_if_present(stage: Usd.Stage, path: str) -> None:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(Sdf.Path(path))


def _z_axis_quat(angle: float) -> Gf.Quatd:
    return Gf.Quatd(
        math.cos(angle / 2.0),
        Gf.Vec3d(0.0, 0.0, math.sin(angle / 2.0)),
    )


def _set_local_pose(prim_path: str, translate: Gf.Vec3d, yaw: float) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(translate)
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(_z_axis_quat(yaw))


def _find_model_part(stage: Usd.Stage, part_name: str) -> Usd.Prim | None:
    suffix = f"/{part_name}"
    for prim in stage.Traverse():
        if prim.GetPath().pathString.endswith(suffix):
            return prim
    return None


def _get_translate_op(xform: UsdGeom.Xformable) -> UsdGeom.XformOp:
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return xform.AddTranslateOp()


def _get_orient_op(xform: UsdGeom.Xformable) -> UsdGeom.XformOp:
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            return op
    return xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)


def _rotate_z(vec: Gf.Vec3d, angle: float) -> Gf.Vec3d:
    c = math.cos(angle)
    s = math.sin(angle)
    return Gf.Vec3d(c * vec[0] - s * vec[1], s * vec[0] + c * vec[1], vec[2])


def _create_preview_material(
    stage: Usd.Stage,
    path: str,
    color: tuple[float, float, float],
) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.42)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*(channel * 0.15 for channel in color))
    )
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _bind_material_to_subtree(prim: Usd.Prim, material: UsdShade.Material) -> int:
    bound_count = 0
    for target in Usd.PrimRange(prim):
        if target.IsA(UsdGeom.Gprim) or target == prim:
            UsdShade.MaterialBindingAPI(target).Bind(material)
            bound_count += 1
    return bound_count


def apply_sorting_part_materials(stage: Usd.Stage) -> None:
    red_material = _create_preview_material(
        stage,
        f"{FINAL_MODEL_PATH}/Materials/SortingRedMat",
        SORTING_RED_COLOR,
    )
    bound_parts: list[str] = []
    for part_name in RED_SORTING_PARTS:
        prim = _find_model_part(stage, part_name)
        if prim is None:
            print(f"[FinalRobot] 找不到红色分拣部件: {part_name}", flush=True)
            continue
        _bind_material_to_subtree(prim, red_material)
        bound_parts.append(part_name)
    print(f"[FinalRobot] 红色分拣部件: {', '.join(bound_parts)}", flush=True)


def apply_home_claw_pose(stage: Usd.Stage) -> None:
    """Set the imported visual links to the open, lifted, claw-roll=-1.57 pose."""
    roll_prim = _find_model_part(stage, "claw_roll")
    if roll_prim is None:
        print("[FinalRobot] 找不到 claw_roll link，无法设置初始夹爪姿态", flush=True)
        return

    roll_xform = UsdGeom.Xformable(roll_prim)
    roll_translate_op = _get_translate_op(roll_xform)
    roll_orient_op = _get_orient_op(roll_xform)
    roll_base_translate_value = roll_translate_op.Get()
    roll_base_translate = (
        Gf.Vec3d(roll_base_translate_value)
        if roll_base_translate_value is not None
        else Gf.Vec3d(0.0, 0.0, 0.0)
    )
    roll_base_orient_value = roll_orient_op.Get()
    roll_base_orient = (
        Gf.Quatd(
            float(roll_base_orient_value.GetReal()),
            Gf.Vec3d(roll_base_orient_value.GetImaginary()),
        )
        if roll_base_orient_value is not None
        else Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
    )

    for part_name in ("claw_roll", "claw_lift", "left_claw", "right_claw"):
        prim = _find_model_part(stage, part_name)
        if prim is None:
            continue
        xform = UsdGeom.Xformable(prim)
        translate_op = _get_translate_op(xform)
        orient_op = _get_orient_op(xform)
        base_translate_value = translate_op.Get()
        base_translate = (
            Gf.Vec3d(base_translate_value)
            if base_translate_value is not None
            else Gf.Vec3d(0.0, 0.0, 0.0)
        )
        base_orient_value = orient_op.Get()
        base_orient = (
            Gf.Quatd(
                float(base_orient_value.GetReal()),
                Gf.Vec3d(base_orient_value.GetImaginary()),
            )
            if base_orient_value is not None
            else Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
        )
        translate_op.Set(
            roll_base_translate
            + _rotate_z(base_translate - roll_base_translate, DEFAULT_CLAW_ROLL_HOME)
        )
        if part_name == "claw_roll":
            orient_op.Set(_z_axis_quat(DEFAULT_CLAW_ROLL_HOME) * roll_base_orient)
        else:
            orient_op.Set(_z_axis_quat(DEFAULT_CLAW_ROLL_HOME) * base_orient)

    print("[FinalRobot] 初始夹爪姿态: claw-roll=-1.57, claw=open", flush=True)


def _place_model_on_field(stage: Usd.Stage, prim_path: str) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return

    xform = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xform.AddTranslateOp()

    current_value = translate_op.Get()
    current = Gf.Vec3d(current_value) if current_value is not None else Gf.Vec3d(0.0, 0.0, 0.0)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(prim).ComputeAlignedBox()
    if bbox.IsEmpty():
        return

    z_delta = FINAL_MODEL_GROUND_Z - float(bbox.GetMin()[2])
    adjusted = Gf.Vec3d(current[0], current[1], current[2] + z_delta)
    translate_op.Set(adjusted)
    print(
        f"[FinalRobot] 贴地高度调整: bbox_min_z={bbox.GetMin()[2]:.4f}, "
        f"offset_z={adjusted[2]:.4f}",
        flush=True,
    )


def import_final_robot_model(stage: Usd.Stage, apply_home_pose: bool = False) -> str:
    """
    Replace the simple block body with the Final URDF visual model.

    The robot control code still moves /World/LogisticsField/Robot. The imported
    model is a child of that root, so it follows the existing path-planning demo
    without changing the control interface or sensor prim paths.
    """
    runtime_urdf = prepare_runtime_urdf()

    _remove_prim_if_present(stage, f"{ROBOT_ROOT_PATH}/Body")
    _remove_prim_if_present(stage, FINAL_MODEL_PATH)
    _remove_prim_if_present(stage, "/Final")
    _remove_prim_if_present(stage, "/World/Final")
    _remove_prim_if_present(stage, "/最后的最后的最后")
    _remove_prim_if_present(stage, "/World/最后的最后的最后")

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig 失败")

    import_config.merge_fixed_joints = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = False
    import_config.create_physics_scene = False
    import_config.collision_from_visuals = False
    import_config.self_collision = False
    import_config.default_drive_strength = 0.0
    import_config.default_position_drive_damping = 0.0

    before_paths = {prim.GetPath().pathString for prim in stage.Traverse()}
    status, imported_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(runtime_urdf),
        import_config=import_config,
        get_articulation_root=False,
    )
    if not status:
        raise RuntimeError("URDFParseAndImportFile 失败")

    imported_path = str(imported_path)
    if not imported_path or not stage.GetPrimAtPath(imported_path):
        after_paths = {prim.GetPath().pathString for prim in stage.Traverse()}
        candidates = sorted(
            path
            for path in after_paths - before_paths
            if path.count("/") <= 2 and path not in ("/World", ROBOT_ROOT_PATH)
        )
        if not candidates:
            raise RuntimeError("无法定位导入后的 Final 机器人 prim")
        imported_path = candidates[0]

    omni.kit.commands.execute(
        "MovePrim",
        path_from=imported_path,
        path_to=FINAL_MODEL_PATH,
    )
    _set_local_pose(FINAL_MODEL_PATH, FINAL_MODEL_OFFSET, FINAL_MODEL_YAW_OFFSET)
    _place_model_on_field(stage, FINAL_MODEL_PATH)
    apply_sorting_part_materials(stage)
    if apply_home_pose:
        apply_home_claw_pose(stage)
    print(f"[FinalRobot] 已导入真实机器人模型: {FINAL_MODEL_PATH}", flush=True)
    return FINAL_MODEL_PATH
