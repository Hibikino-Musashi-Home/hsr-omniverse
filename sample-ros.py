# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

from omni.isaac.kit import SimulationApp

kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
kit.set_setting("/app/extensions/installUntrustedExtensions", True)

import sys
import os
import numpy as np
import xml.etree.ElementTree as ET
import xacro
import omni.ui
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, nucleus
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import omni.kit.commands
from omni.isaac.version import get_version
import hsr
from pxr import Sdf, Gf, UsdPhysics

try:
    import rosgraph
    if not rosgraph.is_master_online():
        print("Please run roscore before executing this script")
        kit.close()
        exit()
except ImportError:
    pass

viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

create_prim(
    "/World/Light_1",
    "SphereLight",
    position=np.array([2.0, 0.0, 5.0]),
    attributes={
        "inputs:radius": 0.01,
        "inputs:intensity": 1e5,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)
create_prim(
    "/World/Light_2",
    "SphereLight",
    position=np.array([-2.0, 0.0, 5.0]),
    attributes={
        "inputs:radius": 0.01,
        "inputs:intensity": 1e5,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)

# Loading the simple_room environment
assets_root_path = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/" + get_version()[0]
BACKGROUND_STAGE_PATH = "/background"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Hospital/hospital.usd"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Office/office.usd"
BACKGROUND_USD_PATH = "/Isaac/Environments/Grid/default_environment.usd"

stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)

# Extract poses of objects from the world file
world_file = os.path.dirname(os.path.abspath(__file__)) + "/tmc_wrs_gazebo/tmc_wrs_gazebo_worlds/worlds/wrs2020.world.xacro"
tree = ET.ElementTree(ET.fromstring(xacro.process_file(world_file, mappings={'trofast_knob': 'true'}).toxml()))
root = tree.getroot()
for i in root.findall('world/include'):
    model_name = i.find('name').text
    model_uri = i.find('uri').text
    (x, y, z, er, ep, ey) = [float(n) for n in i.find('pose').text.split(' ')]
    stage_path = f'/{model_name}'
    model_path = model_uri.replace('model://', os.path.dirname(os.path.abspath(__file__)) + '/usd/wrc_models/') + '/model.usd'
    if not os.path.exists(model_path):
        continue
    create_prim(prim_path=stage_path, prim_type="Xform", translation=[x, y, z], orientation=euler_angles_to_quat([er, ep, ey]))
    stage.add_reference_to_stage(model_path, Sdf.Path(stage_path))
    if i.find('static') is not None:
        # Create fixed joint between the world if the object is static
        root_joint = UsdPhysics.FixedJoint.Define(omni.usd.get_context().get_stage(), stage_path + '/root_joint')
        root_joint.CreateBody1Rel().SetTargets([stage_path + '/link'])
        root_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0))
        root_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        root_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        root_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))


#for i, o in enumerate(['003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle']):
#    path = f'/ycb_{o}'
#    create_prim(prim_path=path, prim_type="Xform", position=[0.3, 0.0, 2.0 + 0.2 * i])
#    stage.add_reference_to_stage(assets_root_path + f'/Isaac/Props/YCB/Axis_Aligned_Physics/{o}.usd', path)

#_hsr = hsr.hsr()

# Start simulation
kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
kit.update()
#_hsr.onsimulationstart(simulation_context)
simulation_context.initialize_physics()
omni.timeline.get_timeline_interface().play()

while kit.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)
    #_hsr.step()

simulation_context.stop()
kit.close()
