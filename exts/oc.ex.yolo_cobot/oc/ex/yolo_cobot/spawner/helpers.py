import random

import numpy as np
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import add_reference_to_stage

import utils.log as log
from utils.path import get_proj_root_path

from .enums import ObjectItem, ObjectType


def spawn(
    task_name: str,
    object_item: ObjectItem,
    total_items: int,
    spawn_point: np.ndarray,
    orient_deg: np.ndarray,
):
    item_usd_path = get_item_usd_path(object_item)
    item_stage_path = create_item_stage_path(task_name, object_item, total_items)

    # object pose

    oriant_quat = euler_angles_to_quat(orient_deg, degrees=True)

    add_reference_to_stage(usd_path=item_usd_path, prim_path=item_stage_path)
    item = XFormPrim(item_stage_path)
    item.set_local_pose(spawn_point, oriant_quat)

    return item


def create_item_stage_path(task_name: str, object_item: ObjectItem, total_items: int):
    name = object_item.name.lower().strip().removeprefix("geo_").removeprefix("real_")
    return f"/Execute/{task_name}/{name.capitalize()}_{total_items}"


def get_item_usd_path(object_item: ObjectItem):
    path = object_item.value
    proj_root = get_proj_root_path()
    return f"{proj_root}/assets/objects/{path}"


def random_object(filter: ObjectType) -> ObjectItem:
    try:
        items = list(ObjectItem)
        if filter == ObjectType.GEOMETRIC:
            items = [
                item for item in ObjectItem if item.name.lower().startswith("geo_")
            ]
        else:
            items = [
                item for item in ObjectItem if item.name.lower().startswith("real_")
            ]

        total = len(items)
        rand_index = random.randint(0, total - 1)
        return items[rand_index]
    except Exception as e:
        log.error(f"random_object error; Error = {e}")
        return ObjectItem.GEO_CUBE
