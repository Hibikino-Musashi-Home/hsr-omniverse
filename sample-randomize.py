# Copyright (c) 2025
# All rights reserved.
#
# YCB ドメインランダム化 + データセット生成
#   - 家具を WRS 配置で固定
#   - YCB をハードコードした家具天板に物理スポーン
#   - 乱択化サイクルごとに複数カメラから RGB + セマンティックセグメンテーションを保存
#   - クラス粒度: YCB の種類ごと (例: ycb_011_banana)
#
# 出力:
#   /data/dataset/{rgb,semantic_segmentation}/ に保存される
#   (docker-compose.dev.yml で /data をマウントしておくこと)
#
# 使い方:
#   /isaac-sim/python.sh /app/sample-ycb-dataset.py

from isaacsim.simulation_app import SimulationApp

kit = SimulationApp({
    "renderer": "PathTracing",
    "headless": True,                    # GUI を切って高速化
    "samples_per_pixel_per_frame": 32,
    "max_bounces": 12,
    "max_specular_transmission_bounces": 12,
    "max_volume_bounces": 6,
})

# Isaac Sim のリアルタイム制限を無効化
kit.set_setting("/app/runLoops/main/rateLimitEnabled", False)

# デノイザ設定
kit.set_setting("/rtx/pathtracing/optixDenoiser/enabled", True)
kit.set_setting("/rtx/newDenoiser/enabled", True)

# PathTracing の追加品質設定
# firefly clamping: 強い反射などで白い点ノイズが出るのを抑制
kit.set_setting("/rtx/pathtracing/fireflyFilter/enabled", True)
kit.set_setting("/rtx/pathtracing/fireflyFilter/maxIntensityPerSample", 50.0)
# キャッシュされたパストレース (時間的に安定したサンプリング)
kit.set_setting("/rtx/pathtracing/cached/enabled", True)
# 露出制御を自動化 (シーンの明るさが変わっても見やすい)
kit.set_setting("/rtx/post/tonemap/op", 1)  # ACES tonemapping

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

# 出力先
OUTPUT_DIR = "/data/dataset"

# データセット枚数 (=乱択化サイクル数)
NUM_ITERATIONS = 50

# 各サイクル内で何枚撮るか (= カメラ位置の数)
# 45° ずつ回転して 8 画角撮る
NUM_CAMERA_VIEWS_PER_ITER = 8

# 画像解像度 (フォトリアル設定)
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960

# 物理落下フレーム数
# physics_dt = 1/60 と組み合わせて約 50 秒分の物理進行
DROP_SETTLE_FRAMES = 3000

# YCB のスポーン
YCB_DROP_MARGIN = 0.3
YCB_SURFACE_INSET = 0.05
YCB_YAW_RANGE = (-np.pi, np.pi)
YCB_SCALE_RANGE = (0.7, 1.3)
YCB_COUNT_RANGE = (4, 10)
# プールサイズ。プールには YCB の全種類 (79 種) を作成して、
# 各サイクルで visible_count 個をランダム選択して表示する。
# これにより 79 種類すべてが満遍なくデータセットに登場する。
# discover_ycb_models() の戻り値長で動的に決定するため、ここで定数は持たない。

# カメラ位置の乱択化範囲 (シーン中央を見る形)
CAM_LOOKAT = (0.0, 0.0, 0.5)
CAM_RADIUS_RANGE = (2.0, 4.5)
CAM_AZIMUTH_RANGE = (0.0, 2.0 * np.pi)
CAM_ELEVATION_RANGE = (np.pi / 12, np.pi / 3)   # 仰角 15度〜60度

# 主要家具の天板位置 (ハードコード)
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

# ラボテクスチャ (床と壁を WRS のデフォルトから差し替える)
FLOOR_TEXTURE = "/data/LabTextures/lab_floor.jpg"
WALL_TEXTURE = "/data/LabTextures/lab_wall.jpg"
FLOOR_TILE = 1.0
WALL_TILE = 1.0

# 物理
PHYSICS_GRAVITY = 9.81

# 照明 (フォトリアル設定)
HDRI_PATH = "/data/HDRIs/events_hall_interior_4k.exr"
HDRI_INTENSITY_RANGE = (2000.0, 3500.0)   # 環境光をしっかり効かせる
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
    # CCD をシーン全体で有効化 (各 RigidBody の enableCCD と組み合わせて使う)
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
    """OmniPBR マテリアルを作成して JPG を貼る (UV 無しメッシュ対応)。
    sample-ros-lab.py と同じ方法で project_uvw=True にして UV 無しでも貼れる。
    """
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


def _list_prim_tree(prim, depth=0, max_depth=4):
    """デバッグ用: プリム階層を表示"""
    if depth > max_depth:
        return
    indent = "  " * depth
    print(f"{indent}{prim.GetPath()} [{prim.GetTypeName()}]")
    for child in prim.GetChildren():
        _list_prim_tree(child, depth + 1, max_depth)


def apply_lab_textures():
    """床と壁にラボテクスチャを適用する。
    重要: この関数は timeline.play() の "後" に呼ばないと HSR の Articulation や
    PhysX のセットアップを壊すことがある (sample-ros-lab.py 知見)。
    """
    _stage = omni.usd.get_context().get_stage()

    # デバッグ: GroundPlane と wrc_frame の階層を確認
    print("[texture] === GroundPlane 階層 ===")
    gp = _stage.GetPrimAtPath("/World/GroundPlane")
    if gp.IsValid():
        _list_prim_tree(gp)
    else:
        print("  /World/GroundPlane が無効")
    print("[texture] === wrc_frame 階層 (visuals まで) ===")
    wf = _stage.GetPrimAtPath("/wrc_frame")
    if wf.IsValid():
        _list_prim_tree(wf, max_depth=3)
    else:
        print("  /wrc_frame が無効")

    # 床: GroundPlane の構造に応じてバインド先を探す
    # AddGroundPlaneCommand は /World/GroundPlane の下に
    #   CollisionMesh [Mesh]   ← レンダリング用
    #   CollisionPlane [Plane] ← コリジョン用
    # を作る。テクスチャは CollisionMesh に貼る必要がある。
    floor_candidates = [
        "/World/GroundPlane/CollisionMesh",
        "/World/GroundPlane/GroundPlane/CollisionMesh",
        "/World/GroundPlane/CollisionPlane",
        "/World/GroundPlane/GroundPlane/CollisionPlane",
        "/World/GroundPlane",
    ]
    applied_floor = False
    for path in floor_candidates:
        if apply_texture(path, FLOOR_TEXTURE, FLOOR_TILE):
            print(f"[texture] 床にテクスチャ適用成功: {path}")
            applied_floor = True
            break
    if not applied_floor:
        print(f"[texture] 床にテクスチャ適用できませんでした (有効なプリムが見つからない)")

    # 壁: /wrc_frame/link/visuals/mesh_* の 5 枚
    wall_parent = _stage.GetPrimAtPath("/wrc_frame/link/visuals")
    if wall_parent.IsValid():
        count = 0
        for child in wall_parent.GetChildren():
            if child.GetName().startswith("mesh_"):
                if apply_texture(str(child.GetPath()), WALL_TEXTURE, WALL_TILE):
                    count += 1
        print(f"[texture] 壁にテクスチャ適用: {count} 枚 ({WALL_TEXTURE})")
    else:
        print(f"[texture] /wrc_frame/link/visuals が見つからない")


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
ycb_class_names = []   # 各 YCB の意味クラス名 (ycb_011_banana など)


def spawn_ycb_pool(ycb_models):
    """YCB の全種類分のプリムを作成してプール化する。
    各サイクルでは visible_count 個だけ表示し、残りは画面外に退避させる。
    """
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

        # CCD (Continuous Collision Detection) を有効化:
        # 高速落下時に薄い天板を擦り抜けるのを防ぐ
        if body_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            physx_rb = PhysxSchema.PhysxRigidBodyAPI(body_prim)
        else:
            physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body_prim)
        physx_rb.CreateEnableCCDAttr().Set(True)

        # セマンティックラベルを付与
        add_update_semantics(prim, semantic_label=name, type_label="class")

        ycb_prims.append(prim)
        ycb_body_prims.append(body_prim)
        ycb_class_names.append(name)
    print(f"[ycb] プール作成: {len(ycb_prims)} 種 (全 YCB、CCD 有効)")


def sample_spawn_pose(surfaces):
    weights = [s["area"] for s in surfaces]
    surf = random.choices(surfaces, weights=weights, k=1)[0]
    x = random.uniform(surf["x_min"], surf["x_max"])
    y = random.uniform(surf["y_min"], surf["y_max"])
    z = surf["top_z"] + YCB_DROP_MARGIN
    return (x, y, z)


def sample_spawn_pose_on_target():
    """ターゲット家具の上にスポーン位置をサンプリング。"""
    if cycle_target_surface is None:
        return (0.0, 0.0, YCB_FLOOR_DROP_HEIGHT)
    s = cycle_target_surface
    x = random.uniform(s["x_min"], s["x_max"])
    y = random.uniform(s["y_min"], s["y_max"])
    z = s["top_z"] + YCB_DROP_MARGIN
    return (x, y, z)


def teleport_and_randomize_ycb(surfaces):
    """ターゲット家具の上にだけ YCB をスポーンする。
    surfaces 引数は互換性のため残しているが、実際は cycle_target_surface を使う。
    """
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
    """全 YCB の線速度・角速度をゼロにして凍結する。

    物体が落ち切らないうちに撮影されたり、転がり続けて止まらないのを
    防ぐために、撮影直前に強制的に動きを止める。
    """
    frozen = 0
    for body in ycb_body_prims:
        if not body.IsValid():
            continue
        if not body.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        rb = UsdPhysics.RigidBodyAPI(body)
        # 線速度
        vel_attr = rb.GetVelocityAttr()
        if not vel_attr:
            vel_attr = rb.CreateVelocityAttr()
        vel_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        # 角速度
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
    """照明を作成:
    - DomeLight に HDRI を貼って環境光・反射を作る (フォトリアル感の主役)
    - SphereLight 数個でハイライト・シャドウを追加
    """
    global light_prims, dome_light_prim
    usd_stage = omni.usd.get_context().get_stage()

    # DomeLight + HDRI
    dome_path = "/World/Lights/Dome"
    dome_light_prim = usd_stage.DefinePrim(dome_path, "DomeLight")
    dome_light_prim.CreateAttribute("inputs:intensity",
                                    Sdf.ValueTypeNames.Float).Set(DOME_LIGHT_INTENSITY)
    # HDRI ファイルを texture:file としてセット
    if os.path.isfile(HDRI_PATH):
        dome_light_prim.CreateAttribute("inputs:texture:file",
                                        Sdf.ValueTypeNames.Asset).Set(HDRI_PATH)
        dome_light_prim.CreateAttribute("inputs:texture:format",
                                        Sdf.ValueTypeNames.Token).Set("latlong")
        print(f"[lights] HDRI 適用: {HDRI_PATH}")
    else:
        print(f"[lights] HDRI が見つかりません: {HDRI_PATH} (DomeLight のみで継続)")

    # SphereLight でハイライトとシャドウを追加
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
    # HDRI の強度を乱択化
    if dome_light_prim is not None and dome_light_prim.IsValid():
        intensity_attr = dome_light_prim.GetAttribute("inputs:intensity")
        if intensity_attr:
            intensity_attr.Set(random.uniform(*HDRI_INTENSITY_RANGE))

    # SphereLight の位置・強度・色温度を乱択化
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
# 各サイクルで「すべての物体を集中させる家具」と「カメラの基準方位角」
cycle_target_surface = None   # surface 辞書 (1 つだけ)
cycle_base_azimuth = 0.0      # 基準方位角


def setup_camera():
    """データセット撮影用カメラを作成。位置・向きは後で乱択化する。"""
    global camera_prim
    usd_stage = omni.usd.get_context().get_stage()
    cam_path = "/World/DatasetCamera"
    camera_prim = usd_stage.DefinePrim(cam_path, "Camera")
    UsdGeom.Xformable(camera_prim).AddTranslateOp().Set(Gf.Vec3d(3.0, 3.0, 2.0))
    UsdGeom.Xformable(camera_prim).AddOrientOp().Set(Gf.Quatf(1.0))
    print(f"[camera] {cam_path}")


def begin_camera_cycle(surfaces):
    """新しい乱択化サイクルの開始時に呼ぶ。
    - 物体配置とカメラの両方で使う「ターゲット家具」を 1 つ選ぶ
    - 4画角の基準方位角を決める
    """
    global cycle_target_surface, cycle_base_azimuth
    if surfaces:
        cycle_target_surface = random.choice(surfaces)
        print(f"  ターゲット家具: {cycle_target_surface['name']}")
    else:
        cycle_target_surface = None
    cycle_base_azimuth = random.uniform(0.0, 2.0 * np.pi)


def randomize_camera_view(view_idx):
    """カメラ位置をサイクル内画角に応じて設定する。
    - 方位角は cycle_base_azimuth + 45° × view_idx
    - ターゲットは cycle_target_surface の天板中心
    """
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

    # 位置を設定
    xf = UsdGeom.Xformable(camera_prim)
    existing = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    if "xformOp:translate" in existing:
        existing["xformOp:translate"].Set(Gf.Vec3d(x, y, z))

    # ターゲットを見るクォータニオンを計算
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

    # Replicator Writer を初期化 (最初に 1 回だけ)
    print(f"[main] 出力先: {OUTPUT_DIR}")
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 重要: capture-on-play を無効化する。
    # デフォルトでは、タイムライン再生中の全フレームでキャプチャされてしまう。
    # これを切ると、rep.orchestrator.step() を明示的に呼んだときだけキャプチャされる。
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

    # 物理開始
    # physics_dt は 1/60 (デフォルト) を維持。これより大きくすると
    # 1 ステップで物体が大きく動いて家具をすり抜けるので、
    # 物理時間は DROP_SETTLE_FRAMES のフレーム数で稼ぐ方針。
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
    # テクスチャ適用後に少しフレームを進めてマテリアル更新を反映させる
    for _ in range(3):
        kit.update()

    print(f"[main] データセット生成開始: {NUM_ITERATIONS} サイクル "
          f"x {NUM_CAMERA_VIEWS_PER_ITER} 画角 = {NUM_ITERATIONS * NUM_CAMERA_VIEWS_PER_ITER} 枚予定")

    for iteration in range(NUM_ITERATIONS):
        if not kit.is_running():
            break
        print(f"[iter] {iteration + 1}/{NUM_ITERATIONS}")

        # まずこのサイクルのターゲット家具を決める (これが物体配置とカメラ両方の基準になる)
        begin_camera_cycle(surfaces)

        # 物体配置と照明を乱択化 (物体は cycle_target_surface の上のみにスポーン)
        randomize_lights()
        teleport_and_randomize_ycb(surfaces)

        # 物理を再生(止まっていた場合は再開)。
        # sim_ctx.play() を毎サイクル呼ぶことで、is_playing=False の状態でも
        # 確実に step() で物理が進むようにする。
        sim_ctx.play()

        # 落下・安定: シミュレーション時刻ベースで進める。
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

        # 撮影前の最終凍結(振動収束のため)
        freeze_ycb_velocities()
        for _ in range(30):
            if not kit.is_running():
                break
            sim_ctx.step(render=False)

        # 物理結果を USD/Hydra に同期
        for _ in range(3):
            if not kit.is_running():
                break
            sim_ctx.step(render=True)
        for _ in range(3):
            kit.update()

        # 撮影前に、出力ディレクトリにある既存ファイル一覧を控える
        # (撮影後に増えたファイルだけを iter 番号付きにリネームするため)
        # ※ BasicWriter は OUTPUT_DIR 直下に rgb_NNNN.png 等を保存する (サブフォルダなし)
        existing_files = set(os.listdir(OUTPUT_DIR)) if os.path.isdir(OUTPUT_DIR) else set()

        # 複数画角で撮影 (方位角を 90° ずつズラして 4 画角)
        for view_idx in range(NUM_CAMERA_VIEWS_PER_ITER):
            if not kit.is_running():
                break
            randomize_camera_view(view_idx)
            # カメラを動かしたあと数フレーム update してから撮影
            for _ in range(5):
                kit.update()
            # Replicator にレンダー & 保存させる
            # rt_subframes: PathTracing のサブフレーム数。
            # samples_per_pixel_per_frame=32 × rt_subframes=256 = 8192 sample/pixel
            # OptiX デノイザと合わせて完璧な品質を狙う。
            rep.orchestrator.step(rt_subframes=256)

        # Replicator がすべての画像をディスクに書き終わるまで待つ
        # (rep.orchestrator.step は非同期で、データの書き出しは後で行われるため)
        rep.orchestrator.wait_until_complete()

        # 撮影で新規追加されたファイルを iter 番号付きにリネーム
        iter_tag = f"iter{iteration + 1:03d}"
        new_files = sorted(set(os.listdir(OUTPUT_DIR)) - existing_files)
        # rgb / semantic_segmentation / bbox をそれぞれ識別してリネーム
        rgb_files = [f for f in new_files if f.startswith("rgb_") and f.endswith(".png")]
        seg_files = [f for f in new_files if f.startswith("semantic_segmentation_") and f.endswith(".png")]
        json_files = [f for f in new_files if f.startswith("semantic_segmentation_labels_") and f.endswith(".json")]
        # bbox 2D tight 関連:
        #   bounding_box_2d_tight_NNNN.npy           ... bbox 配列
        #   bounding_box_2d_tight_labels_NNNN.json   ... semantic_id -> クラス名
        #   bounding_box_2d_tight_prim_paths_NNNN.json ... semantic_id -> prim path
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

        # 物体 (ycb_*) が写っていない画像を削除する
        # 各 view の semseg labels JSON を見て、ycb_ で始まるクラスが含まれているか確認
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
            # label_dict は {"(R,G,B,A)": {"class": "ycb_011_banana"}, ...} の形式
            has_ycb = False
            for v in label_dict.values():
                cls = v.get("class", "") if isinstance(v, dict) else str(v)
                if cls.startswith("ycb_"):
                    has_ycb = True
                    break
            if not has_ycb:
                # この view は物体が写っていない → 関連ファイル全部を削除
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