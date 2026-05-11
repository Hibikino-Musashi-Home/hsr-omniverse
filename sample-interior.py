# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.
#
# Minimal example: InteriorAgent scene + HSR robot (no WRS objects)
#
# 使い方:
#   1. 単独で実行する場合(デフォルトのシーン kujiale_0003 を使用):
#        /isaac-sim/python.sh /app/sample-interior.py
#
#   2. 環境変数 INTERIOR_AGENT_SCENE で別のシーンを指定:
#        INTERIOR_AGENT_SCENE=kujiale_0007 /isaac-sim/python.sh /app/sample-interior.py
#
#   3. interior_agent/run.sh から起動(推奨):
#        ./interior_agent/run.sh kujiale_0003

import os

# ============================================================
# 設定(シーンや HSR 配置の調整はここ)
# ============================================================

# 使用するシーン名(環境変数 INTERIOR_AGENT_SCENE があればそれを優先)
DEFAULT_SCENE_NAME = "kujiale_0003"
SCENE_NAME = os.environ.get("INTERIOR_AGENT_SCENE", DEFAULT_SCENE_NAME)

# InteriorAgent のマウント先(コンテナ内パス)
INTERIOR_AGENT_BASE = "/data/InteriorAgent"

# HSR の初期配置(シーンに合わせて調整)
HSR_TRANSLATION = [0.0, 0.0, 0.0]   # [X, Y, Z]
HSR_YAW = 0.0                        # 向き(ラジアン)

# 補助照明の強度(InteriorAgent には自前の照明があるので、補助は最小限。0 で無効化)
EXTRA_LIGHT_INTENSITY = 3e4

# カメラ視点
CAMERA_EYE = [5.0, 5.0, 3.0]
CAMERA_TARGET = [0.0, 0.0, 1.0]

# ============================================================
# Isaac Sim の起動
# ============================================================

from isaacsim.simulation_app import SimulationApp

kit = SimulationApp({
    "renderer": "RayTracedLighting",
    "headless": False,
    "extra_args": [
        "--/app/extensions/excluded/0=isaacsim.asset.importer.urdf",
        "--/app/extensions/excluded/1=isaacsim.ros2.urdf",
    ],
})
kit.set_setting("/app/extensions/installUntrustedExtensions", True)

# Isaac Sim 起動後にしかインポートできないモジュール
import math
import numpy as np

from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import omni.kit.commands
import omni.timeline
import omni.usd

from pxr import Sdf, Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema, PhysicsSchemaTools

import hsr

# ============================================================
# シーンパスの解決
# ============================================================

scene_usda = f"{INTERIOR_AGENT_BASE}/{SCENE_NAME}/{SCENE_NAME}.usda"

if not os.path.isfile(scene_usda):
    print(f"[sample-interior] エラー: シーンファイルが見つかりません: {scene_usda}")
    print(f"[sample-interior] 利用可能なシーンを確認するには:")
    print(f"[sample-interior]   ls {INTERIOR_AGENT_BASE}/")
    kit.close()
    raise SystemExit(1)

print(f"[sample-interior] シーン: {SCENE_NAME}")
print(f"[sample-interior] USD: {scene_usda}")

# ============================================================
# カメラ視点
# ============================================================

viewports.set_camera_view(
    eye=np.array(CAMERA_EYE),
    target=np.array(CAMERA_TARGET),
)

# ============================================================
# 補助照明
# ============================================================

if EXTRA_LIGHT_INTENSITY > 0:
    create_prim(
        "/World/Light_Main",
        "SphereLight",
        position=np.array([0.0, 0.0, 5.0]),
        attributes={
            "inputs:radius": 0.5,
            "inputs:intensity": EXTRA_LIGHT_INTENSITY,
            "inputs:color": (1.0, 1.0, 1.0),
        },
    )

# ============================================================
# InteriorAgent シーンの読み込み
# ============================================================

BACKGROUND_STAGE_PATH = "/background"
stage.add_reference_to_stage(scene_usda, BACKGROUND_STAGE_PATH)

# ============================================================
# HSR の配置
# ============================================================

hsr_stage_path = "/hsrb"
create_prim(
    prim_path=hsr_stage_path,
    prim_type="Xform",
    translation=HSR_TRANSLATION,
    orientation=euler_angles_to_quat([0, 0, HSR_YAW]),
)
_hsr = hsr.hsr(stage_path=hsr_stage_path)

print(f"[sample-interior] HSR 配置: translation={HSR_TRANSLATION}, yaw={HSR_YAW}")

# ============================================================
# シミュレーション開始
# ============================================================

kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
kit.update()
_hsr.onsimulationstart(simulation_context)
simulation_context.initialize_physics()
omni.timeline.get_timeline_interface().play()

print("[sample-interior] シミュレーション開始")

# ============================================================
# メインループ
# ============================================================

while kit.is_running():
    simulation_context.step(render=True)
    _hsr.step()

simulation_context.stop()
kit.close()
