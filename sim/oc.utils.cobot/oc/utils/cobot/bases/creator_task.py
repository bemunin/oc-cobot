from typing import Optional

import numpy as np
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import add_reference_to_stage


class CreatorTask(BaseTask):
    def __init__(
        self,
        name: str,
        offset: Optional[np.ndarray] = None,
        item_per_row: Optional[int] = 1,
        num_row: Optional[int] = 1,
        item_spacing: Optional[float] = 0,
        row_spacing: Optional[float] = 0,
    ):
        super().__init__(name, offset)

        self._item_per_row = item_per_row
        self._item_spacing = item_spacing
        self._row_spacing = row_spacing
        self._num_row = num_row

    def _import_obj(
        self,
        usd_path: str,
        prim_path: str,
        name: str,
        position: Optional[np.ndarray] = [0.0, 0.0, 0.0],
        orientation_deg: Optional[np.ndarray] = [0.0, 0.0, 0.0],
        scale: Optional[np.ndarray] = [1.0, 1.0, 1.0],
        prim_type: Optional[str] = "XFormPrim",
    ):
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

        if not prim_utils.get_prim_at_path(prim_path):
            prim_utils.create_prim(
                prim_path=prim_path, prim_type="Xform", position=position
            )

        orientation = euler_angles_to_quat(orientation_deg, degrees=True)
        if prim_type == "RigidPrim":
            return RigidPrim(
                prim_path=prim_path,
                name=name,
                scale=scale,
                position=position,
                orientation=orientation,
            )
        else:
            return XFormPrim(
                prim_path=prim_path,
                name=name,
                scale=scale,
                position=position,
                orientation=orientation,
            )
