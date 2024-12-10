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
from omni.isaac.core.materials.physics_material import PhysicsMaterial

from tmc_wrs_gazebo_worlds import randomizer

is_ros2 = False
try:
    import rclpy
    from gazebo_msgs.srv import GetWorldProperties, GetModelState
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

viewports.set_camera_view(eye=np.array([3.7, 1.7, 5.0]), target=np.array([0, 0, 0]))

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

# adjust friction of the floor
floor_material = PhysicsMaterial(
    prim_path='/Floor',
    static_friction=60.0,
    dynamic_friction=60.0)
omni.kit.commands.execute('BindMaterialExt',
                            material_path='/Floor',
                            prim_path=[BACKGROUND_STAGE_PATH + '/GroundPlane/CollisionPlane'],
                            strength=['weakerThanDescendants'],
                            material_purpose='physics')

model_names = []

# Extract poses of objects from the world file
if is_ros2:
    world_file = "/ws/install/tmc_wrs_gazebo_worlds/share/tmc_wrs_gazebo_worlds/worlds/wrs2020_knob.world"
else:
    world_file = "/opt/ros/noetic/share/tmc_wrs_gazebo_worlds/worlds/wrs2020_knob.world"
tree = ET.parse(world_file)
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

if is_ros2:
    import std_msgs.msg
    collision_detect_pub = _hsr.ros2node.create_publisher(std_msgs.msg.Bool, '/undesired_contact_detector/detect', qos_profile=rclpy.qos.qos_profile_system_default)
else:
    import std_msgs.msg
    collision_detect_pub = rospy.Publisher('/undesired_contact_detector/detect', std_msgs.msg.Bool, queue_size=10)

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


actor_to_body_name_cache = {}
def contact_report_event(ch, cd):
    for c in ch:
        try:
            body1 = actor_to_body_name_cache[c.actor1]
        except KeyError:
            body1 = str(PhysicsSchemaTools.intToSdfPath(c.actor1)).split('/')[1]
            actor_to_body_name_cache[c.actor1] = body1
        if body1 != 'background':
            print(f'Contact {body1}')
        if body1 == 'wrc_frame' or body1.startswith('task2a_'):
            collision_detect_pub.publish(std_msgs.msg.Bool(True))

# this variable is unused, but it is required to continue the subscription
_contact_report_event_sub = get_physx_simulation_interface().subscribe_contact_report_events(contact_report_event)

# Start simulation
kit.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)
kit.update()
_hsr.onsimulationstart(simulation_context)
simulation_context.initialize_physics()
omni.timeline.get_timeline_interface().play()

# simulate gazebo ros APIs required for task evaluators
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

if is_ros2:
    def handle_get_world_properties_ros2(req, ret):
        ret.model_names = model_names
        ret.success = True
        return ret

    def handle_get_model_state_ros2(req, ret):
        stage = omni.usd.get_context().get_stage()
        objxform = get_xform(stage, req.model_name)
        refxform = get_xform(stage, req.relative_entity_name)
        relpose = refxform.GetInverse() * objxform
        translation = relpose.ExtractTranslation()
        rotation = relpose.GetOrthonormalized().ExtractRotationQuat()
        rotation_imaginary = rotation.GetImaginary()
        # create response
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

    _hsr.ros2node.create_service(GetWorldProperties, '/gazebo/get_world_properties', handle_get_world_properties_ros2, qos_profile=rclpy.qos.qos_profile_system_default)
    _hsr.ros2node.create_service(GetModelState, '/gazebo/get_model_state', handle_get_model_state_ros2, qos_profile=rclpy.qos.qos_profile_system_default)
else:
    def handle_get_world_properties(req):
        ret = GetWorldPropertiesResponse()
        ret.model_names = model_names
        ret.success = True
        return ret

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

    rospy.Service('/gazebo/get_world_properties', GetWorldProperties, handle_get_world_properties)
    rospy.Service('/gazebo/get_model_state', GetModelState, handle_get_model_state)

while kit.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)
    _hsr.step()

simulation_context.stop()
kit.close()
