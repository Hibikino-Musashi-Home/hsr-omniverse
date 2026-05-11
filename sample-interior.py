# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.
#
# Minimal example: InteriorAgent scene + HSR robot (no WRS objects)

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

# --- カメラ視点 ---
viewports.set_camera_view(eye=np.array([5.0, 5.0, 3.0]), target=np.array([0, 0, 1]))

# --- 照明(InteriorAgent 自前の照明があるので最小限) ---
create_prim(
    "/World/Light_Main",
    "SphereLight",
    position=np.array([0.0, 0.0, 5.0]),
    attributes={
        "inputs:radius": 0.5,
        "inputs:intensity": 3e4,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)

# --- InteriorAgent シーンの読み込み ---
INTERIOR_AGENT_PATH = "/data/InteriorAgent/kujiale_0003/kujiale_0003.usda"
BACKGROUND_STAGE_PATH = "/background"
stage.add_reference_to_stage(INTERIOR_AGENT_PATH, BACKGROUND_STAGE_PATH)

# --- HSR の配置(座標は試行錯誤) ---
hsr_stage_path = "/hsrb"
create_prim(
    prim_path=hsr_stage_path,
    prim_type="Xform",
    translation=[-1.6, 0.0, 0.0],    # ← 試行錯誤するパラメータ
    orientation=euler_angles_to_quat([0, 0, 0])
)
_hsr = hsr.hsr(stage_path=hsr_stage_path)

# --- シミュレーション開始 ---
kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
kit.update()
_hsr.onsimulationstart(simulation_context)
simulation_context.initialize_physics()
omni.timeline.get_timeline_interface().play()

# --- メインループ ---
while kit.is_running():
    simulation_context.step(render=True)
    _hsr.step()

simulation_context.stop()
kit.close()
