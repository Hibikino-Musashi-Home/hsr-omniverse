# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

from omni.isaac.kit import SimulationApp

kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
kit.set_setting("/app/extensions/installUntrustedExtensions", True)

import numpy as np
import omni.ui
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, nucleus
import omni.kit.commands
import rosgraph
import hsr


if not rosgraph.is_master_online():
    print("Please run roscore before executing this script")
    kit.close()
    exit()

viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the simple_room environment
assets_root_path = nucleus.get_assets_root_path()
BACKGROUND_STAGE_PATH = "/background"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

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
