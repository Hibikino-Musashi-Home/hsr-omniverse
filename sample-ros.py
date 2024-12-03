# Copyright (c) 2023, Toyota Motor Corporation
# Copyright (c) 2023, MID Academic Promotions, Inc.
# All rights reserved.

from omni.isaac.kit import SimulationApp

kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
kit.set_setting("/app/extensions/installUntrustedExtensions", True)

import sys
import os
import math
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
from pxr import Sdf, Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from omni.physx import get_physx_simulation_interface
from omni.isaac.sensor import ContactSensor
from omni.isaac.sensor import _sensor

sys.path.append(os.path.dirname(__file__) + '/tmc_wrs_gazebo/tmc_wrs_gazebo_worlds/src')

from tmc_wrs_gazebo_worlds import randomizer

is_ros2 = False
try:
    import rclpy
    from gazebo_msgs.srv import GetWorldProperties, GetWorldProperties_Response as GetWorldPropertiesResponse, GetModelState, GetModelState_Response as GetModelStateResponse
    is_ros2 = True
except ImportError:
    import rospy
    from gazebo_msgs.srv import GetWorldProperties, GetWorldPropertiesResponse, GetModelState, GetModelStateResponse

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
        "inputs:intensity": 5e4,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)
create_prim(
    "/World/Light_2",
    "SphereLight",
    position=np.array([-2.0, 0.0, 5.0]),
    attributes={
        "inputs:radius": 0.01,
        "inputs:intensity": 5e4,
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

model_names = []

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
    model_names.append(model_name)


def drop_object(gazebo_name, name, x, y, z, yaw):
    global model_names
    print(f'Drop {name} ({x}, {y}, {z}, {yaw})')
    stage_path = f'/{gazebo_name.replace("-", "_")}'
    model_path = os.path.dirname(os.path.abspath(__file__)) + '/usd/wrc_models/' + name + '/model.usd'
    if not os.path.exists(model_path):
        return
    create_prim(prim_path=stage_path, prim_type="Xform", translation=[x, y, z], orientation=euler_angles_to_quat([0, 0, yaw]))
    stage.add_reference_to_stage(model_path, Sdf.Path(stage_path))
    model_names.append(gazebo_name)


randomizer.generate_wrs_task(drop_func=drop_object)

hsr_stage_path = "/hsrb"
create_prim(prim_path=hsr_stage_path, prim_type="Xform", translation=[-2.1, 1.2, 0], orientation=euler_angles_to_quat([0, 0, -math.pi/2]))

_hsr = hsr.hsr(stage_path=hsr_stage_path)

contact_links = [
    "/hsrb/hsrb/base_link/collisions",
    "/hsrb/hsrb/base_f_bumper_link/collisions",
    "/hsrb/hsrb/base_b_bumper_link/collisions"
]

contact_sensors = []
stage_handle = omni.usd.get_context().get_stage()
for i in range(len(contact_links)):
    contact_report_api = PhysxSchema.PhysxContactReportAPI.Apply(stage_handle.GetPrimAtPath(contact_links[i]))
    contact_report_api.CreateThresholdAttr(0.0)
    contact_sensors.append(ContactSensor(
        prim_path=f'{contact_links[i]}/Contact_Sensor',
        name="Contact_Sensor",
        frequency=10,
        min_threshold=0,
        radius=-1
    ))


def contact_report_event(ch, cd):
    for c in ch:
        body1 = str(PhysicsSchemaTools.intToSdfPath(c.actor1)).split('/')[1]
        if body1 != 'background':
            print(f'Contact {body1}')


get_physx_simulation_interface().subscribe_contact_report_events(contact_report_event)

# Start simulation
kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
kit.update()
_hsr.onsimulationstart(simulation_context)
simulation_context.initialize_physics()
omni.timeline.get_timeline_interface().play()

# simulate gazebo ros APIs required for task evaluators
def handle_get_world_properties(req):
    ret = GetWorldPropertiesResponse()
    ret.model_names = model_names
    ret.success = True
    return ret

def get_xform(stage, model_name):
    try:
        name = model_name.replace('::link', '').replace('-', '_')
        prim = stage.GetPrimAtPath(f'/{name}/link')
        if not prim.IsValid():
            prim = stage.GetPrimAtPath(f'/{name}/body')
        if not prim.IsValid():
            prim = stage.GetPrimAtPath(f'/{name}/hsrb')
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    except:
        #print(f'Failed to get xform for {model_name}')
        return Gf.Matrix4d()

def handle_get_model_state(req):
    stage = omni.usd.get_context().get_stage()
    objxform = get_xform(stage, req.model_name)
    refxform = get_xform(stage, req.relative_entity_name)
    relpose = refxform.GetInverse() * objxform
    translation = relpose.ExtractTranslation()
    rotation = relpose.GetOrthonormalized().ExtractRotationQuat()
    rotation_imaginary = rotation.GetImaginary()
    # create response
    ret = GetModelStateResponse()
    ret.header.frame_id = req.relative_entity_name
    ret.pose.position.x = translation[0]
    ret.pose.position.y = translation[1]
    ret.pose.position.z = translation[2]
    ret.pose.orientation.x = rotation_imaginary[0]
    ret.pose.orientation.y = rotation_imaginary[1]
    ret.pose.orientation.z = rotation_imaginary[2]
    ret.pose.orientation.w = rotation.GetReal()
    ret.success = True
    return ret

if is_ros2:
    _hsr.ros2node.create_service('/gazebo/get_world_properties', GetWorldProperties, handle_get_world_properties)
    _hsr.ros2node.create_service('/gazebo/get_model_state', GetModelState, handle_get_model_state)
else:
    rospy.Service('/gazebo/get_world_properties', GetWorldProperties, handle_get_world_properties)
    rospy.Service('/gazebo/get_model_state', GetModelState, handle_get_model_state)

while kit.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)
    _hsr.step()

simulation_context.stop()
kit.close()
