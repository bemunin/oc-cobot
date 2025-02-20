import numpy as np
import omni.isaac.core.utils.extensions as ext_utils
import omni.isaac.core.utils.numpy.rotations as rot_utils


def ext_assets_path():
    ext_id = ext_utils.get_extension_id("oc.scene.cobot_conveyor")
    ext_path = ext_utils.get_extension_path(ext_id)
    return f"{ext_path}/assets"


def to_quaternion(euler_angle_degs: list[float]) -> np.ndarray:
    return rot_utils.euler_angles_to_quats(
        euler_angle_degs,
        degrees=True,
    )
