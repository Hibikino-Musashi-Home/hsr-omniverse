# Copyright (c) 2025
# All rights reserved.
#
# YCB ドメインランダム化 + データセット生成
#   - 家具を WRS 配置で固定
#   - YCB をハードコードした家具天板に物理スポーン
#   - 乱択化サイクルごとに複数カメラから RGB + セマンティックセグメンテーション + bbox を保存
#   - クラス粒度: YCB の種類ごと (例: ycb_011_banana)
#   - 環境ボックス: シーン全体を 4 枚の壁写真で取り囲む (UV 付きカスタムメッシュ)
#
# 出力:
#   /data/dataset/ 直下に iter{NNN}_view{VV}_{rgb,semseg,bbox,...} を保存

from isaacsim.simulation_app import SimulationApp

kit = SimulationApp({
    "renderer": "PathTracing",
    "headless": False,
    "samples_per_pixel_per_frame": 32,
    "max_bounces": 12,
    "max_specular_transmission_bounces": 12,
    "max_volume_bounces": 6,
})

kit.set_setting("/app/runLoops/main/rateLimitEnabled", False)
kit.set_setting("/rtx/pathtracing/optixDenoiser/enabled", True)
kit.set_setting("/rtx/newDenoiser/enabled", True)
kit.set_setting("/rtx/pathtracing/fireflyFilter/enabled", True)
kit.set_setting("/rtx/pathtracing/fireflyFilter/maxIntensityPerSample", 50.0)
kit.set_setting("/rtx/pathtracing/cached/enabled", True)
kit.set_setting("/rtx/post/tonemap/op", 1)

import os
import glob
import json
import math
import random
import xml.etree.ElementTree as ET
import numpy as np

import omni.usd
import omni.kit.commands
import omni.timeline
import omni.replicator.core as rep
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Gf

from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.semantics import add_update_semantics


# ============================================================
# 設定
# ============================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WRS_MODELS_ROOT = os.path.join(SCRIPT_DIR, "usd", "wrs_models")

OUTPUT_DIR = "/data/dataset"
NUM_ITERATIONS = 50
NUM_CAMERA_VIEWS_PER_ITER = 8
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960
DROP_SETTLE_FRAMES = 3000

YCB_DROP_MARGIN = 0.3
YCB_SURFACE_INSET = 0.05
YCB_YAW_RANGE = (-np.pi, np.pi)
YCB_SCALE_RANGE = (0.7, 1.3)
YCB_COUNT_RANGE = (4, 10)

CAM_LOOKAT = (0.0, 0.0, 0.5)
CAM_RADIUS_RANGE = (2.0, 4.5)
CAM_AZIMUTH_RANGE = (0.0, 2.0 * np.pi)
CAM_ELEVATION_RANGE = (np.pi / 12, np.pi / 3)

HARDCODED_SURFACES = [
    {"name": "wrc_long_table_center",   "center_x": -0.3, "center_y":  0.2,  "top_z": 0.75, "size_x": 0.6, "size_y": 1.4},
    {"name": "wrc_long_table_wall",     "center_x": -2.7, "center_y": -0.3,  "top_z": 0.75, "size_x": 0.6, "size_y": 1.4},
    {"name": "wrc_tall_table",          "center_x": -0.3, "center_y":  1.2,  "top_z": 0.95, "size_x": 0.4, "size_y": 0.4},
    {"name": "wrc_stair_drawer_top",    "center_x": -2.7, "center_y":  1.0,  "top_z": 0.50, "size_x": 0.4, "size_y": 0.8},
    {"name": "wrc_bookshelf_mid",       "center_x":  2.7, "center_y": -1.0,  "top_z": 1.00, "size_x": 0.7, "size_y": 0.25},
    {"name": "trofast_1",               "center_x": -2.7, "center_y":  0.67, "top_z": 0.33, "size_x": 0.4, "size_y": 0.2},
    {"name": "trofast_2",               "center_x": -2.7, "center_y":  1.0,  "top_z": 0.33, "size_x": 0.4, "size_y": 0.2},
    {"name": "trofast_3",               "center_x": -2.7, "center_y":  1.0,  "top_z": 0.59, "size_x": 0.4, "size_y": 0.2},
    {"name": "wrc_tray_1",              "center_x": -2.7, "center_y": -0.75, "top_z": 0.42, "size_x": 0.25, "size_y": 0.15},
    {"name": "wrc_tray_2",              "center_x": -2.7, "center_y": -0.45, "top_z": 0.42, "size_x": 0.25, "size_y": 0.15},
]

# ラボテクスチャ
FLOOR_TEXTURE = "/data/LabTextures/lab_floor.jpg"
# 環境ボックス用の 4 枚の壁テクスチャ
ENV_WALL_TEXTURES = [
    "/data/LabTextures/lab_wall_1.jpg",
    "/data/LabTextures/lab_wall_2.jpg",
    "/data/LabTextures/lab_wall_3.jpg",
    "/data/LabTextures/lab_wall_4.jpg",
]
FLOOR_TILE = 1.0

# 環境ボックス (シーン全体を取り囲む大きな部屋)
ENV_ROOM_SIZE = 15.0
ENV_ROOM_HEIGHT = 4.0

PHYSICS_GRAVITY = 9.81

HDRI_PATH = "/data/HDRIs/events_hall_interior_4k.exr"
HDRI_INTENSITY_RANGE = (2000.0, 3500.0)
DOME_LIGHT_INTENSITY = 2500.0
LIGHT_COUNT = 3
LIGHT_INTENSITY_RANGE = (3e4, 8e4)
LIGHT_COLOR_TEMP_RANGE = (3500.0, 7500.0)
LIGHT_HEIGHT_RANGE = (3.0, 6.0)
LIGHT_XY_RANGE = (-4.0, 4.0)


# ============================================================
# ユーティリティ
# ============================================================

def find_model_usd(model_name):
    candidates = [
        os.path.join(WRS_MODELS_ROOT, model_name, "model.usd"),
        os.path.join(WRS_MODELS_ROOT, model_name, f"{model_name}.usd"),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return None


def discover_ycb_models():
    pattern = os.path.join(WRS_MODELS_ROOT, "ycb_*")
    found = []
    for d in sorted(glob.glob(pattern)):
        name = os.path.basename(d)
        usd = find_model_usd(name)
        if usd is not None:
            found.append((name, usd))
    return found


def _get_or_add_op(xformable, suffix, op_type, precision):
    key = f"xformOp:{suffix}"
    existing = {op.GetOpName(): op for op in xformable.GetOrderedXformOps()}
    if key in existing:
        return existing[key]
    return xformable.AddXformOp(op_type, precision)


def set_pose(prim, translate=None, rotate_xyz=None, scale=None):
    xf = UsdGeom.Xformable(prim)
    if translate is not None:
        op = _get_or_add_op(xf, "translate",
                            UsdGeom.XformOp.TypeTranslate,
                            UsdGeom.XformOp.PrecisionDouble)
        op.Set(Gf.Vec3d(*translate))
    if rotate_xyz is not None:
        op = _get_or_add_op(xf, "rotateXYZ",
                            UsdGeom.XformOp.TypeRotateXYZ,
                            UsdGeom.XformOp.PrecisionDouble)
        op.Set(Gf.Vec3d(*rotate_xyz))
    if scale is not None:
        op = _get_or_add_op(xf, "scale",
                            UsdGeom.XformOp.TypeScale,
                            UsdGeom.XformOp.PrecisionDouble)
        op.Set(Gf.Vec3d(*scale))


def set_visible(prim, visible):
    img = UsdGeom.Imageable(prim)
    if visible:
        img.MakeVisible()
    else:
        img.MakeInvisible()


def apply_collision_to_all_meshes(root_prim, approximation=None):
    count = 0
    for desc in Usd.PrimRange(root_prim):
        if desc.IsA(UsdGeom.Mesh) or desc.IsA(UsdGeom.Gprim):
            if not desc.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(desc)
            if approximation is not None and desc.IsA(UsdGeom.Mesh):
                if not desc.HasAPI(UsdPhysics.MeshCollisionAPI):
                    mca = UsdPhysics.MeshCollisionAPI.Apply(desc)
                else:
                    mca = UsdPhysics.MeshCollisionAPI(desc)
                mca.CreateApproximationAttr().Set(approximation)
            count += 1
    return count


# ============================================================
# 基本シーン
# ============================================================

def setup_physics_scene():
    usd_stage = omni.usd.get_context().get_stage()
    scene_path = "/World/PhysicsScene"
    scene = UsdPhysics.Scene.Define(usd_stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(PHYSICS_GRAVITY)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(usd_stage.GetPrimAtPath(scene_path))
    physx_scene.CreateEnableCCDAttr().Set(True)


def setup_ground_plane():
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=omni.usd.get_context().get_stage(),
        planePath="/World/GroundPlane",
        axis="Z",
        size=20.0,
        position=Gf.Vec3f(0, 0, 0),
        color=Gf.Vec3f(0.5, 0.5, 0.5),
    )


def apply_texture(prim_path, texture_path, tile):
    """既存の OmniPBR ベースのテクスチャ適用 (床用)。"""
    _stage = omni.usd.get_context().get_stage()
    if not _stage.GetPrimAtPath(prim_path).IsValid():
        print(f"[texture] プリムが見つかりません: {prim_path}")
        return False
    if not os.path.isfile(texture_path):
        print(f"[texture] テクスチャが見つかりません: {texture_path}")
        return False
    print(f"[texture] 適用中: {prim_path} ← {texture_path}")
    mtl_created = []
    omni.kit.commands.execute(
        "CreateAndBindMdlMaterialFromLibrary",
        mdl_name="OmniPBR.mdl", mtl_name="OmniPBR",
        mtl_created_list=mtl_created, bind_selected_prims=False,
    )
    mtl_prim = _stage.GetPrimAtPath(mtl_created[0])
    omni.usd.create_material_input(mtl_prim, "diffuse_texture",
        Sdf.AssetPath(texture_path), Sdf.ValueTypeNames.Asset)
    omni.usd.create_material_input(mtl_prim, "texture_scale",
        Gf.Vec2f(tile, tile), Sdf.ValueTypeNames.Float2)
    omni.usd.create_material_input(mtl_prim, "project_uvw",
        True, Sdf.ValueTypeNames.Bool)
    omni.usd.create_material_input(mtl_prim, "world_or_object",
        True, Sdf.ValueTypeNames.Bool)
    UsdShade.MaterialBindingAPI(_stage.GetPrimAtPath(prim_path)).Bind(
        UsdShade.Material(mtl_prim),
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )
    return True


def create_env_wall(prim_path, width, height, position, rotation_euler_deg, texture_path):
    """環境ボックスの 1 面分の壁を UV 付きカスタムメッシュとして作成し、
    UsdPreviewSurface + UsdUVTexture でテクスチャを 1 枚だけ貼る。

    壁はローカルで X-Z 平面上に幅 width、高さ height の長方形として作る。
    その後、translate と rotateXYZ でワールドに配置する。
    """
    _stage = omni.usd.get_context().get_stage()

    if not os.path.isfile(texture_path):
        print(f"[env] テクスチャが見つかりません: {texture_path}")
        return False

    # === UV 付き 4 頂点メッシュを作る ===
    mesh = UsdGeom.Mesh.Define(_stage, prim_path)

    half_w = width / 2.0
    half_h = height / 2.0
    points = [
        Gf.Vec3f(-half_w, 0.0, -half_h),   # 左下
        Gf.Vec3f( half_w, 0.0, -half_h),   # 右下
        Gf.Vec3f( half_w, 0.0,  half_h),   # 右上
        Gf.Vec3f(-half_w, 0.0,  half_h),   # 左上
    ]
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([
        Gf.Vec3f(0.0, 1.0, 0.0),
        Gf.Vec3f(0.0, 1.0, 0.0),
        Gf.Vec3f(0.0, 1.0, 0.0),
        Gf.Vec3f(0.0, 1.0, 0.0),
    ])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    # UV (primvar:st)
    primvars_api = UsdGeom.PrimvarsAPI(mesh)
    st_primvar = primvars_api.CreatePrimvar(
        "st",
        Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.vertex,
    )
    st_primvar.Set([
        Gf.Vec2f(0.0, 0.0),
        Gf.Vec2f(1.0, 0.0),
        Gf.Vec2f(1.0, 1.0),
        Gf.Vec2f(0.0, 1.0),
    ])

    # === 配置 (translate + rotate) ===
    xf = UsdGeom.Xformable(mesh.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*position))
    xf.AddRotateXYZOp().Set(Gf.Vec3d(*rotation_euler_deg))

    # === UsdPreviewSurface + UsdUVTexture でテクスチャを貼る ===
    mat_path = f"{prim_path}_mat"
    material = UsdShade.Material.Define(_stage, mat_path)

    shader = UsdShade.Shader.Define(_stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    uv_reader = UsdShade.Shader.Define(_stage, mat_path + "/UVReader")
    uv_reader.CreateIdAttr("UsdPrimvarReader_float2")
    uv_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    uv_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(_stage, mat_path + "/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_path))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        uv_reader.ConnectableAPI(), "result"
    )
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb"
    )
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
    )

    print(f"[env] 壁作成: {prim_path} size=({width:.1f}×{height:.1f}m)"
          f" pos={position} rot={rotation_euler_deg}")
    print(f"       tex={texture_path}")
    return True


def _list_prim_tree(prim, depth=0, max_depth=4):
    if depth > max_depth:
        return
    indent = "  " * depth
    print(f"{indent}{prim.GetPath()} [{prim.GetTypeName()}]")
    for child in prim.GetChildren():
        _list_prim_tree(child, depth + 1, max_depth)


def apply_lab_textures():
    """床と環境ボックスにラボテクスチャを適用する。
    重要: timeline.play() の "後" に呼ぶこと (PhysX セットアップを壊さないため)。
    """
    _stage = omni.usd.get_context().get_stage()

    # 床: GroundPlane の CollisionMesh にバインド
    floor_candidates = [
        "/World/GroundPlane/CollisionMesh",
        "/World/GroundPlane/GroundPlane/CollisionMesh",
        "/World/GroundPlane/CollisionPlane",
        "/World/GroundPlane/GroundPlane/CollisionPlane",
        "/World/GroundPlane",
    ]
    for path in floor_candidates:
        if apply_texture(path, FLOOR_TEXTURE, FLOOR_TILE):
            print(f"[texture] 床にテクスチャ適用成功: {path}")
            break

    # 環境ボックス: シーン全体を 4 枚の壁で取り囲む
    # 壁は X-Z 平面のローカル長方形として作り、Z 軸まわりに回転で各方角に向ける。
    # 法線(+Y)が内側を向くように回転する。
    half = ENV_ROOM_SIZE / 2.0
    h = ENV_ROOM_HEIGHT

    walls = [
        # 北壁: y = +half、法線を -Y に向けたい → Z 軸 180° 回転
        ("/World/EnvBox/Wall_N", ENV_ROOM_SIZE, h,
         (0.0,  half, h / 2.0), (0.0, 0.0, 180.0),
         ENV_WALL_TEXTURES[0]),
        # 南壁: y = -half、法線が +Y (デフォルト)
        ("/World/EnvBox/Wall_S", ENV_ROOM_SIZE, h,
         (0.0, -half, h / 2.0), (0.0, 0.0,   0.0),
         ENV_WALL_TEXTURES[1]),
        # 東壁: x = +half、法線を -X に向けたい → Z 軸 +90° 回転
        ("/World/EnvBox/Wall_E", ENV_ROOM_SIZE, h,
         ( half, 0.0, h / 2.0), (0.0, 0.0,  90.0),
         ENV_WALL_TEXTURES[2]),
        # 西壁: x = -half、法線を +X に向けたい → Z 軸 -90° 回転
        ("/World/EnvBox/Wall_W", ENV_ROOM_SIZE, h,
         (-half, 0.0, h / 2.0), (0.0, 0.0, -90.0),
         ENV_WALL_TEXTURES[3]),
    ]

    create_prim(prim_path="/World/EnvBox", prim_type="Xform")
    for path, w, ht, pos, rot, tex in walls:
        create_env_wall(path, w, ht, pos, rot, tex)
    print(f"[env] 環境ボックス作成完了: {ENV_ROOM_SIZE}m × {ENV_ROOM_SIZE}m × {h}m")


# ============================================================
# 家具
# ============================================================

WORLD_FILE_CANDIDATES = [
    "/ws/install/tmc_wrs_gazebo_worlds/share/tmc_wrs_gazebo_worlds/worlds/wrs2020_knob.world",
    "/opt/ros/noetic/share/tmc_wrs_gazebo_worlds/worlds/wrs2020_knob.world",
]


def find_world_file():
    for p in WORLD_FILE_CANDIDATES:
        if os.path.isfile(p):
            return p
    return None


def spawn_furniture_from_world():
    world_file = find_world_file()
    if world_file is None:
        print(f"[furniture] WRS .world が見つかりません")
        return []

    spawned = []
    tree = ET.parse(world_file)
    root = tree.getroot()
    for i in root.findall("world/include"):
        model_name = i.find("name").text
        model_uri = i.find("uri").text
        (x, y, z, er, ep, ey) = [float(n) for n in i.find("pose").text.split(" ")]
        prim_path = f"/{model_name}"
        model_path = model_uri.replace("model://",
                                       WRS_MODELS_ROOT + "/") + "/model.usd"
        if not os.path.exists(model_path):
            continue
        create_prim(prim_path=prim_path, prim_type="Xform",
                    translation=[x, y, z],
                    orientation=euler_angles_to_quat([er, ep, ey]))
        stage.add_reference_to_stage(model_path, Sdf.Path(prim_path))

        if i.find("static") is not None:
            usd_stage = omni.usd.get_context().get_stage()
            joint = UsdPhysics.FixedJoint.Define(usd_stage,
                                                 prim_path + "/root_joint")
            joint.CreateBody1Rel().SetTargets([prim_path + "/link"])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        spawned.append(prim_path)
    print(f"[furniture] {len(spawned)} 体配置")
    return spawned


def apply_collision_to_furniture(furniture_paths):
    usd_stage = omni.usd.get_context().get_stage()
    for path in furniture_paths:
        prim = usd_stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        apply_collision_to_all_meshes(prim, approximation="meshSimplification")


def build_hardcoded_surfaces():
    surfaces = []
    inset = YCB_SURFACE_INSET
    for s in HARDCODED_SURFACES:
        half_x = s["size_x"] / 2.0
        half_y = s["size_y"] / 2.0
        x_min = s["center_x"] - half_x + inset
        x_max = s["center_x"] + half_x - inset
        y_min = s["center_y"] - half_y + inset
        y_max = s["center_y"] + half_y - inset
        if x_min >= x_max or y_min >= y_max:
            continue
        surfaces.append({
            "name": s["name"],
            "x_min": x_min, "x_max": x_max,
            "y_min": y_min, "y_max": y_max,
            "top_z": s["top_z"],
            "area": (x_max - x_min) * (y_max - y_min),
        })
    print(f"[surface] {len(surfaces)} 面")
    return surfaces


# ============================================================
# YCB
# ============================================================

ycb_prims = []
ycb_body_prims = []
ycb_class_names = []


def spawn_ycb_pool(ycb_models):
    """YCB の全種類分のプリムを作成してプール化する。"""
    global ycb_prims, ycb_body_prims, ycb_class_names
    if not ycb_models:
        return
    usd_stage = omni.usd.get_context().get_stage()
    for i, (name, usd_path) in enumerate(ycb_models):
        prim_path = f"/World/YCB/ycb_{i:03d}"
        create_prim(prim_path=prim_path, prim_type="Xform")
        stage.add_reference_to_stage(usd_path, Sdf.Path(prim_path))
        prim = usd_stage.GetPrimAtPath(prim_path)
        apply_collision_to_all_meshes(prim, approximation="convexHull")

        body_path = prim_path + "/body"
        body_prim = usd_stage.GetPrimAtPath(body_path)
        if not body_prim.IsValid():
            body_prim = prim

        if body_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            physx_rb = PhysxSchema.PhysxRigidBodyAPI(body_prim)
        else:
            physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body_prim)
        physx_rb.CreateEnableCCDAttr().Set(True)

        add_update_semantics(prim, semantic_label=name, type_label="class")

        ycb_prims.append(prim)
        ycb_body_prims.append(body_prim)
        ycb_class_names.append(name)
    print(f"[ycb] プール作成: {len(ycb_prims)} 種 (全 YCB、CCD 有効)")


def sample_spawn_pose_on_target():
    if cycle_target_surface is None:
        return (0.0, 0.0, YCB_DROP_MARGIN)
    s = cycle_target_surface
    x = random.uniform(s["x_min"], s["x_max"])
    y = random.uniform(s["y_min"], s["y_max"])
    z = s["top_z"] + YCB_DROP_MARGIN
    return (x, y, z)


def teleport_and_randomize_ycb(surfaces):
    if not ycb_prims:
        return
    visible_count = random.randint(*YCB_COUNT_RANGE)
    visible_count = min(visible_count, len(ycb_prims))
    visible_idx = set(random.sample(range(len(ycb_prims)), visible_count))

    for i, prim in enumerate(ycb_prims):
        body = ycb_body_prims[i]
        if i in visible_idx:
            set_visible(prim, True)
            x, y, z = sample_spawn_pose_on_target()
            roll_deg  = math.degrees(random.uniform(-np.pi, np.pi))
            pitch_deg = math.degrees(random.uniform(-np.pi, np.pi))
            yaw_deg   = math.degrees(random.uniform(*YCB_YAW_RANGE))
            s = random.uniform(*YCB_SCALE_RANGE)
            set_pose(prim, scale=(s, s, s))
            set_pose(body,
                     translate=(x, y, z),
                     rotate_xyz=(roll_deg, pitch_deg, yaw_deg))
        else:
            set_visible(prim, False)
            set_pose(body, translate=(0.0, 0.0, -50.0))


def freeze_ycb_velocities():
    frozen = 0
    for body in ycb_body_prims:
        if not body.IsValid():
            continue
        if not body.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        rb = UsdPhysics.RigidBodyAPI(body)
        vel_attr = rb.GetVelocityAttr()
        if not vel_attr:
            vel_attr = rb.CreateVelocityAttr()
        vel_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        ang_attr = rb.GetAngularVelocityAttr()
        if not ang_attr:
            ang_attr = rb.CreateAngularVelocityAttr()
        ang_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        frozen += 1
    return frozen


# ============================================================
# 照明
# ============================================================

light_prims = []
dome_light_prim = None


def spawn_lights():
    global light_prims, dome_light_prim
    usd_stage = omni.usd.get_context().get_stage()

    dome_path = "/World/Lights/Dome"
    dome_light_prim = usd_stage.DefinePrim(dome_path, "DomeLight")
    dome_light_prim.CreateAttribute("inputs:intensity",
                                    Sdf.ValueTypeNames.Float).Set(DOME_LIGHT_INTENSITY)
    if os.path.isfile(HDRI_PATH):
        dome_light_prim.CreateAttribute("inputs:texture:file",
                                        Sdf.ValueTypeNames.Asset).Set(HDRI_PATH)
        dome_light_prim.CreateAttribute("inputs:texture:format",
                                        Sdf.ValueTypeNames.Token).Set("latlong")
        print(f"[lights] HDRI 適用: {HDRI_PATH}")
    else:
        print(f"[lights] HDRI が見つかりません: {HDRI_PATH}")

    for i in range(LIGHT_COUNT):
        path = f"/World/Lights/Sphere_{i}"
        lp = usd_stage.DefinePrim(path, "SphereLight")
        UsdGeom.Xformable(lp).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 5.0))
        lp.CreateAttribute("inputs:radius", Sdf.ValueTypeNames.Float).Set(0.1)
        lp.CreateAttribute("inputs:intensity",
                           Sdf.ValueTypeNames.Float).Set(3e4)
        lp.CreateAttribute("inputs:enableColorTemperature",
                           Sdf.ValueTypeNames.Bool).Set(True)
        lp.CreateAttribute("inputs:colorTemperature",
                           Sdf.ValueTypeNames.Float).Set(6500.0)
        light_prims.append(lp)


def randomize_lights():
    if dome_light_prim is not None and dome_light_prim.IsValid():
        intensity_attr = dome_light_prim.GetAttribute("inputs:intensity")
        if intensity_attr:
            intensity_attr.Set(random.uniform(*HDRI_INTENSITY_RANGE))

    for lp in light_prims:
        lp.GetAttribute("inputs:intensity").Set(
            random.uniform(*LIGHT_INTENSITY_RANGE))
        lp.GetAttribute("inputs:colorTemperature").Set(
            random.uniform(*LIGHT_COLOR_TEMP_RANGE))
        xf = UsdGeom.Xformable(lp)
        existing = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
        if "xformOp:translate" in existing:
            existing["xformOp:translate"].Set(Gf.Vec3d(
                random.uniform(*LIGHT_XY_RANGE),
                random.uniform(*LIGHT_XY_RANGE),
                random.uniform(*LIGHT_HEIGHT_RANGE),
            ))


# ============================================================
# カメラ
# ============================================================

camera_prim = None
cycle_target_surface = None
cycle_base_azimuth = 0.0


def setup_camera():
    global camera_prim
    usd_stage = omni.usd.get_context().get_stage()
    cam_path = "/World/DatasetCamera"
    camera_prim = usd_stage.DefinePrim(cam_path, "Camera")
    UsdGeom.Xformable(camera_prim).AddTranslateOp().Set(Gf.Vec3d(3.0, 3.0, 2.0))
    UsdGeom.Xformable(camera_prim).AddOrientOp().Set(Gf.Quatf(1.0))
    print(f"[camera] {cam_path}")


def begin_camera_cycle(surfaces):
    global cycle_target_surface, cycle_base_azimuth
    if surfaces:
        cycle_target_surface = random.choice(surfaces)
        print(f"  ターゲット家具: {cycle_target_surface['name']}")
    else:
        cycle_target_surface = None
    cycle_base_azimuth = random.uniform(0.0, 2.0 * np.pi)


def randomize_camera_view(view_idx):
    radius = random.uniform(*CAM_RADIUS_RANGE)
    azimuth = cycle_base_azimuth + view_idx * (np.pi / 4.0)
    elevation = random.uniform(*CAM_ELEVATION_RANGE)

    if cycle_target_surface is not None:
        s = cycle_target_surface
        cx = (s["x_min"] + s["x_max"]) / 2.0
        cy = (s["y_min"] + s["y_max"]) / 2.0
        cz = s["top_z"] + 0.15
    else:
        cx, cy, cz = CAM_LOOKAT

    x = cx + radius * math.cos(elevation) * math.cos(azimuth)
    y = cy + radius * math.cos(elevation) * math.sin(azimuth)
    z = cz + radius * math.sin(elevation)

    xf = UsdGeom.Xformable(camera_prim)
    existing = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    if "xformOp:translate" in existing:
        existing["xformOp:translate"].Set(Gf.Vec3d(x, y, z))

    eye = Gf.Vec3d(x, y, z)
    target = Gf.Vec3d(cx, cy, cz)
    up = Gf.Vec3d(0, 0, 1)
    look_at_quatd = (Gf.Matrix4d()
                     .SetLookAt(eye, target, up)
                     .GetInverse()
                     .ExtractRotation()
                     .GetQuat())
    if "xformOp:orient" in existing:
        existing["xformOp:orient"].Set(Gf.Quatf(look_at_quatd))


# ============================================================
# メイン
# ============================================================

def main():
    setup_physics_scene()
    setup_ground_plane()

    furniture_paths = spawn_furniture_from_world()
    apply_collision_to_furniture(furniture_paths)

    ycb_models = discover_ycb_models()
    print(f"[main] YCB 候補: {len(ycb_models)} 種")
    spawn_ycb_pool(ycb_models)

    spawn_lights()
    setup_camera()

    surfaces = build_hardcoded_surfaces()

    print(f"[main] 出力先: {OUTPUT_DIR}")
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    rep.orchestrator.set_capture_on_play(False)

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=OUTPUT_DIR,
        rgb=True,
        semantic_segmentation=True,
        colorize_semantic_segmentation=True,
        bounding_box_2d_tight=True,
    )
    render_product = rep.create.render_product(
        str(camera_prim.GetPath()),
        (IMAGE_WIDTH, IMAGE_HEIGHT),
    )
    writer.attach(render_product)
    print(f"[main] Writer attach 完了 (解像度 {IMAGE_WIDTH}x{IMAGE_HEIGHT})")

    kit.update()
    sim_ctx = SimulationContext(
        stage_units_in_meters=1.0,
        physics_dt=1.0/60.0,
        rendering_dt=1.0/60.0,
    )
    kit.update()
    sim_ctx.initialize_physics()
    omni.timeline.get_timeline_interface().play()

    # ラボテクスチャを適用 (timeline.play() の後でないと PhysX セットアップを壊す)
    apply_lab_textures()
    for _ in range(3):
        kit.update()

    print(f"[main] データセット生成開始: {NUM_ITERATIONS} サイクル "
          f"x {NUM_CAMERA_VIEWS_PER_ITER} 画角 = {NUM_ITERATIONS * NUM_CAMERA_VIEWS_PER_ITER} 枚予定")

    for iteration in range(NUM_ITERATIONS):
        if not kit.is_running():
            break
        print(f"[iter] {iteration + 1}/{NUM_ITERATIONS}")

        begin_camera_cycle(surfaces)
        randomize_lights()
        teleport_and_randomize_ycb(surfaces)

        sim_ctx.play()

        SETTLE_DURATION_SEC = 5.0
        start_time = sim_ctx.current_time
        print(f"  落下開始: start_time={start_time:.3f}, "
              f"physics_dt={sim_ctx.get_physics_dt():.4f}, "
              f"is_playing={sim_ctx.is_playing()}")
        max_steps = 10000
        step_count = 0
        while step_count < max_steps:
            if not kit.is_running():
                break
            elapsed = sim_ctx.current_time - start_time
            if elapsed >= SETTLE_DURATION_SEC:
                break
            sim_ctx.step(render=False)
            step_count += 1
            if step_count == 1 or step_count == 100:
                print(f"    step={step_count}: current_time={sim_ctx.current_time:.4f}, "
                      f"elapsed={elapsed:.4f}")
        print(f"  落下: {step_count} ステップで "
              f"{sim_ctx.current_time - start_time:.2f}秒 経過")

        freeze_ycb_velocities()
        for _ in range(30):
            if not kit.is_running():
                break
            sim_ctx.step(render=False)

        for _ in range(3):
            if not kit.is_running():
                break
            sim_ctx.step(render=True)
        for _ in range(3):
            kit.update()

        existing_files = set(os.listdir(OUTPUT_DIR)) if os.path.isdir(OUTPUT_DIR) else set()

        for view_idx in range(NUM_CAMERA_VIEWS_PER_ITER):
            if not kit.is_running():
                break
            randomize_camera_view(view_idx)
            for _ in range(5):
                kit.update()
            rep.orchestrator.step(rt_subframes=256)

        rep.orchestrator.wait_until_complete()

        iter_tag = f"iter{iteration + 1:03d}"
        new_files = sorted(set(os.listdir(OUTPUT_DIR)) - existing_files)
        rgb_files = [f for f in new_files if f.startswith("rgb_") and f.endswith(".png")]
        seg_files = [f for f in new_files if f.startswith("semantic_segmentation_") and f.endswith(".png")]
        json_files = [f for f in new_files if f.startswith("semantic_segmentation_labels_") and f.endswith(".json")]
        bbox_npy_files = [f for f in new_files if f.startswith("bounding_box_2d_tight_") and f.endswith(".npy")]
        bbox_label_files = [f for f in new_files if f.startswith("bounding_box_2d_tight_labels_") and f.endswith(".json")]
        bbox_prim_files = [f for f in new_files if f.startswith("bounding_box_2d_tight_prim_paths_") and f.endswith(".json")]

        for i, fname in enumerate(rgb_files):
            new_name = f"{iter_tag}_view{i:02d}_rgb.png"
            os.rename(os.path.join(OUTPUT_DIR, fname), os.path.join(OUTPUT_DIR, new_name))
        for i, fname in enumerate(seg_files):
            new_name = f"{iter_tag}_view{i:02d}_semseg.png"
            os.rename(os.path.join(OUTPUT_DIR, fname), os.path.join(OUTPUT_DIR, new_name))
        for i, fname in enumerate(json_files):
            new_name = f"{iter_tag}_view{i:02d}_semseg_labels.json"
            os.rename(os.path.join(OUTPUT_DIR, fname), os.path.join(OUTPUT_DIR, new_name))
        for i, fname in enumerate(bbox_npy_files):
            new_name = f"{iter_tag}_view{i:02d}_bbox.npy"
            os.rename(os.path.join(OUTPUT_DIR, fname), os.path.join(OUTPUT_DIR, new_name))
        for i, fname in enumerate(bbox_label_files):
            new_name = f"{iter_tag}_view{i:02d}_bbox_labels.json"
            os.rename(os.path.join(OUTPUT_DIR, fname), os.path.join(OUTPUT_DIR, new_name))
        for i, fname in enumerate(bbox_prim_files):
            new_name = f"{iter_tag}_view{i:02d}_bbox_prim_paths.json"
            os.rename(os.path.join(OUTPUT_DIR, fname), os.path.join(OUTPUT_DIR, new_name))

        deleted = 0
        for view_idx in range(len(rgb_files)):
            labels_path = os.path.join(OUTPUT_DIR,
                                       f"{iter_tag}_view{view_idx:02d}_semseg_labels.json")
            if not os.path.isfile(labels_path):
                continue
            try:
                with open(labels_path, "r") as f:
                    label_dict = json.load(f)
            except Exception as e:
                print(f"  WARN: {labels_path} 読み込み失敗: {e}")
                continue
            has_ycb = False
            for v in label_dict.values():
                cls = v.get("class", "") if isinstance(v, dict) else str(v)
                if cls.startswith("ycb_"):
                    has_ycb = True
                    break
            if not has_ycb:
                for suffix in ["_rgb.png", "_semseg.png", "_semseg_labels.json",
                               "_bbox.npy", "_bbox_labels.json", "_bbox_prim_paths.json"]:
                    p = os.path.join(OUTPUT_DIR,
                                     f"{iter_tag}_view{view_idx:02d}{suffix}")
                    if os.path.isfile(p):
                        os.remove(p)
                deleted += 1
        print(f"  保存: rgb={len(rgb_files)}, semseg={len(seg_files)}, "
              f"labels={len(json_files)}, bbox={len(bbox_npy_files)}, "
              f"削除(物体なし)={deleted}")

    print(f"[main] 完了。{OUTPUT_DIR} を確認してください。")
    sim_ctx.stop()
    kit.close()


if __name__ == "__main__":
    main()