from typing import List

import omni.isaac.core.utils.bounds as bounds_utils


def get_object_size(prim_path: str) -> List:
    """get the size of the object

    Args:
        prim_path (str): stage path to the object

    Returns:
        List: [width, length, height] of the object in meter
    """

    cache = bounds_utils.create_bbox_cache()
    box = bounds_utils.compute_aabb(cache, prim_path)

    min_x = box[0]
    min_y = box[1]
    min_z = box[2]
    max_x = box[3]
    max_y = box[4]
    max_z = box[5]

    width_m = abs(max_x - min_x)
    length_m = abs(max_y - min_y)

    height_m = abs(max_z - min_z)

    return [width_m, length_m, height_m]
