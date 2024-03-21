# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

from omni.isaac.kit import SimulationApp

kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
kit.set_setting("/app/extensions/installUntrustedExtensions", True)

import sys
import numpy as np
import omni.ui
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, nucleus
from omni.isaac.core.utils.prims import create_prim
import omni.kit.commands
from omni.isaac.version import get_version
import hsr

try:
    import rosgraph
    if not rosgraph.is_master_online():
        print("Please run roscore before executing this script")
        kit.close()
        exit()
except ImportError:
    pass

viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the simple_room environment
assets_root_path = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/" + get_version()[0]
BACKGROUND_STAGE_PATH = "/background"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Hospital/hospital.usd"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Office/office.usd"

stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

for i, o in enumerate(['003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle']):
    path = f'/ycb_{o}'
    create_prim(prim_path=path, prim_type="Xform", position=[0.3, 0.0, 2.0 + 0.2 * i])
    stage.add_reference_to_stage(assets_root_path + f'/Isaac/Props/YCB/Axis_Aligned_Physics/{o}.usd', path)

_hsr = hsr.hsr()

# Start simulation
kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
kit.update()
_hsr.onsimulationstart(simulation_context)
simulation_context.initialize_physics()
omni.timeline.get_timeline_interface().play()

while kit.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)
    _hsr.step()

simulation_context.stop()
kit.close()
