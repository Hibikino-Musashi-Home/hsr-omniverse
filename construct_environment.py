#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Tuple

import omni.kit.commands
import omni.usd
from omni.isaac.core.utils import stage, viewports
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from pxr import Sdf


OBJECT_COLLECTION_PATH = "/World/YcbObjects"
ENV_ROOT = "/World/Env"

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
MODEL_ROOT = os.path.join(REPO_ROOT, "usd", "wrs_models")


def log(message: str) -> None:
    print(f"[construct] {message}")


def _prim_exists(prim_path: str) -> bool:
    stage_handle = omni.usd.get_context().get_stage()
    prim = stage_handle.GetPrimAtPath(prim_path)
    return prim.IsValid()


def ensure_xform(prim_path: str) -> None:
    if not _prim_exists(prim_path):
        create_prim(prim_path, "Xform")


def setup_camera() -> None:
    viewports.set_camera_view(
        eye=(3.0, 1.5, 3.0),
        target=(0.0, 0.0, 0.8),
    )


def setup_lights() -> None:
    if not _prim_exists("/World/LightKey"):
        create_prim(
            "/World/LightKey",
            "SphereLight",
            translation=(2.0, 0.0, 5.0),
            attributes={
                "inputs:radius": 0.01,
                "inputs:intensity": 4.5e4,
            },
        )

    if not _prim_exists("/World/FillLight"):
        create_prim(
            "/World/FillLight",
            "SphereLight",
            translation=(-2.0, -1.5, 5.0),
            attributes={
                "inputs:radius": 0.01,
                "inputs:intensity": 3.0e4,
            },
        )


def setup_object_collection(object_collection_path: str = OBJECT_COLLECTION_PATH) -> None:
    ensure_xform(object_collection_path)


def get_model_usd_path(model_name: str) -> str:
    candidates = [
        os.path.join(MODEL_ROOT, model_name, "model.usd"),
        os.path.join(MODEL_ROOT, model_name, "model.model.usd"),
        os.path.join(MODEL_ROOT, model_name, f"{model_name}.usd"),
    ]

    for path in candidates:
        if os.path.exists(path):
            return path

    raise FileNotFoundError(
        f"USD not found for model '{model_name}'. Checked:\n" +
        "\n".join(candidates)
    )


def add_reference(
    usd_path: str,
    prim_path: str,
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> str:
    if not os.path.exists(usd_path):
        raise FileNotFoundError(f"USD not found: {usd_path}")

    if _prim_exists(prim_path):
        omni.kit.commands.execute("DeletePrims", paths=[prim_path])

    create_prim(
        prim_path,
        "Xform",
        translation=translation,
        orientation=euler_angles_to_quat(rotation_rpy),
    )
    stage.add_reference_to_stage(usd_path, Sdf.Path(prim_path))
    return prim_path


def add_model(
    model_name: str,
    prim_name: Optional[str] = None,
    pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> str:
    usd_path = get_model_usd_path(model_name)
    final_prim_name = prim_name or model_name
    prim_path = f"{ENV_ROOT}/{final_prim_name}"
    return add_reference(
        usd_path=usd_path,
        prim_path=prim_path,
        translation=pos,
        rotation_rpy=rpy,
    )


# wrs2020.world.xacro 相当の配置
WRS_LAYOUT: List[Dict[str, object]] = [
    {"name": "wrc_frame",             "pos": (0.0,   0.0, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_bookshelf",         "pos": (2.7,  -1.0, 0.0), "rpy": (0.0, 0.0, -1.57)},
    {"name": "wrc_bin_black",         "pos": (-2.7, -1.7, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_bin_green",         "pos": (-2.7, -1.2, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_stair_like_drawer", "pos": (-2.7,  1.0, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "trofast",               "pos": (-2.7,  0.67, 0.1), "rpy": (0.0, 0.0, 0.0), "prim_name": "trofast_1"},
    {"name": "trofast",               "pos": (-2.7,  1.0,  0.1), "rpy": (0.0, 0.0, 0.0), "prim_name": "trofast_2"},
    {"name": "trofast",               "pos": (-2.7,  1.0,  0.36),"rpy": (0.0, 0.0, 0.0), "prim_name": "trofast_3"},
    {"name": "wrc_tall_table",        "pos": (-0.3,  1.2, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_long_table",        "pos": (-0.3,  0.2, 0.0), "rpy": (0.0, 0.0,  1.57)},
    {"name": "wrc_long_table",        "pos": (-2.7, -0.3, 0.0), "rpy": (0.0, 0.0,  1.57), "prim_name": "wrc_long_table_0"},
    {"name": "wrc_tray",              "pos": (-2.7, -0.75, 0.4), "rpy": (0.0, 0.0, 1.57), "prim_name": "wrc_tray_1"},
    {"name": "wrc_tray",              "pos": (-2.7, -0.45, 0.4), "rpy": (0.0, 0.0, 1.57), "prim_name": "wrc_tray_2"},
    {"name": "wrc_container_a",       "pos": (-2.7, -0.2, 0.4), "rpy": (0.0, 0.0,  0.0)},
    {"name": "wrc_container_b",       "pos": (-2.7,  0.1, 0.4), "rpy": (0.0, 0.0,  0.0)},
    {"name": "person_standing",       "pos": (0.7,   1.5, 0.0), "rpy": (0.0, 0.0,  0.0)},
    {"name": "person_standing",       "pos": (1.8,   1.5, 0.0), "rpy": (0.0, 0.0,  0.0), "prim_name": "person_standing_0"},
]


def setup_ground_plane() -> None:
    """
    wrs_models 側に ground plane USD がないので、
    Isaac 側で単純な床を作る。
    """
    ground_path = "/World/GroundPlane"
    if _prim_exists(ground_path):
        return

    create_prim(
        prim_path=ground_path,
        prim_type="Cube",
        translation=(0.0, 0.0, -0.005),
        scale=(10.0, 10.0, 0.01),
    )


def setup_wrs_world() -> None:
    ensure_xform(ENV_ROOT)
    setup_ground_plane()

    for entry in WRS_LAYOUT:
        model_name = str(entry["name"])
        prim_name = entry.get("prim_name")
        pos = entry["pos"]
        rpy = entry["rpy"]

        try:
            add_model(
                model_name=model_name,
                prim_name=str(prim_name) if prim_name is not None else None,
                pos=pos,   # type: ignore[arg-type]
                rpy=rpy,   # type: ignore[arg-type]
            )
            log(f"Added environment model: {model_name}")
        except Exception as ex:
            log(f"Failed to add environment model '{model_name}': {ex}")


def construct_environment(
    object_collection_path: str = OBJECT_COLLECTION_PATH,
) -> None:
    ensure_xform("/World")
    setup_camera()
    setup_lights()
    setup_wrs_world()
    setup_object_collection(object_collection_path)


def spawn_object(
    model_name: str,
    x_offset: float,
    y_offset: float,
    z_offset: float = 0.01,
    yaw: float = math.pi / 2.0,
    object_collection_path: str = OBJECT_COLLECTION_PATH,
) -> str:
    usd_path = get_model_usd_path(model_name)

    ensure_xform(object_collection_path)

    stage_path = f"{object_collection_path}/{model_name}"
    if _prim_exists(stage_path):
        omni.kit.commands.execute("DeletePrims", paths=[stage_path])

    create_prim(
        stage_path,
        "Xform",
        translation=(0.39 + x_offset, 0.11 + y_offset, z_offset),
        orientation=euler_angles_to_quat([0.0, 0.0, yaw]),
    )
    stage.add_reference_to_stage(usd_path, Sdf.Path(stage_path))
    return stage_path


def spawn_object_at_pose(
    model_name: str,
    translation: Tuple[float, float, float],
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    object_collection_path: str = OBJECT_COLLECTION_PATH,
    prim_name: Optional[str] = None,
) -> str:
    usd_path = get_model_usd_path(model_name)

    ensure_xform(object_collection_path)

    final_prim_name = prim_name or model_name
    stage_path = f"{object_collection_path}/{final_prim_name}"

    if _prim_exists(stage_path):
        omni.kit.commands.execute("DeletePrims", paths=[stage_path])

    create_prim(
        stage_path,
        "Xform",
        translation=translation,
        orientation=euler_angles_to_quat(rpy),
    )
    stage.add_reference_to_stage(usd_path, Sdf.Path(stage_path))
    return stage_path


def delete_object(stage_path: str) -> None:
    if _prim_exists(stage_path):
        omni.kit.commands.execute("DeletePrims", paths=[stage_path])


def clear_spawned_objects(
    object_collection_path: str = OBJECT_COLLECTION_PATH,
) -> None:
    stage_handle = omni.usd.get_context().get_stage()
    parent_prim = stage_handle.GetPrimAtPath(object_collection_path)
    if not parent_prim.IsValid():
        return

    child_paths = [str(child.GetPath()) for child in parent_prim.GetChildren()]
    if child_paths:
        omni.kit.commands.execute("DeletePrims", paths=child_paths)
