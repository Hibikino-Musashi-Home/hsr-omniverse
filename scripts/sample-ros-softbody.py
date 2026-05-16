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
from omni.physx.scripts import deformableUtils, physicsUtils
import omni.usd
from pxr import UsdGeom, UsdLux, Gf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni.kit.commands
import rosgraph
import hsr


if not rosgraph.is_master_online():
    print("Please run roscore before executing this script")
    kit.close()
    exit()

scene = UsdPhysics.Scene.Define(omni.usd.get_context().get_stage(), "/World/physics")
PhysxSchema.PhysxSceneAPI.Apply(omni.usd.get_context().get_stage().GetPrimAtPath("/World/physics"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(omni.usd.get_context().get_stage(), "/World/physics")
physxSceneAPI.CreateEnableGPUDynamicsAttr(True)

viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Loading the simple_room environment
assets_root_path = nucleus.get_assets_root_path()
BACKGROUND_STAGE_PATH = "/World/background"
#BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)


def create_jello_cube(stage, cube_path, position, size, mesh_path, phys_material_path, grfx_material):
    stage.DefinePrim(cube_path).GetReferences().AddReference(mesh_path)
    skinMesh = UsdGeom.Mesh.Define(stage, cube_path)
    skinMesh.AddTranslateOp().Set(position)
    skinMesh.AddOrientOp().Set(Gf.Quatf(1.0))
    skinMesh.AddScaleOp().Set(Gf.Vec3f(size, size, size))
    deformableUtils.add_physx_deformable_body(
        stage,
        cube_path,
        simulation_hexahedral_resolution=3,
        collision_simplification=True,
        self_collision=False,
        solver_position_iteration_count=20,
    )
    deformableUtils.add_deformable_body_material(
        stage,
        phys_material_path,
        youngs_modulus=10000.0,
        poissons_ratio=0.49,
        damping_scale=0.0,
        dynamic_friction=0.5,
    )
    physicsUtils.add_physics_material_to_prim(stage, skinMesh.GetPrim(), phys_material_path)


# Create a deformable body material and set it on the deformable body
deformable_material_path = omni.usd.get_stage_next_free_path(omni.usd.get_context().get_stage(), "/deformableBodyMaterial", True)
create_jello_cube(
    omni.usd.get_context().get_stage(),
    "/World/jello_box",
    Gf.Vec3f(0.0, 0.0, 10.0),
    0.2,
    demo.get_demo_asset_path("FrankaDeformable/SubUSDs/box_high.usd"),
    deformable_material_path,
    "/redGlass"
)

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
