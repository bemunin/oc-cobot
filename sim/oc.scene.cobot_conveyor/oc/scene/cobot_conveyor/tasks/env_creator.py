import numpy as np
from oc.utils.cobot.bases.creator_task import CreatorTask
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.menu import set_camera_view
from omni.isaac.sensor import Camera

from ..utils import ext_assets_path, to_quaternion


class EnvCreator(CreatorTask):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        container_usd_path = "Collected_Container_C19_61x40x18cm_PR_V_NVD_01/Container_C19_61x40x18cm_PR_V_NVD_01.usd"

        set_camera_view(
            eye=[1.5, -4, 5],
            target=[-0.7, 0, 0.6],
        )

        add_reference_to_stage(
            usd_path=f"{ext_assets_path()}/base_env.usda",
            prim_path="/World/Environment",
        )

        self._topdown_cam = Camera(
            prim_path="/World/SceneCameras/TopDownCamera",
            position=np.array([-0.74, 0, 18]),
            orientation=to_quaternion([0, 90, 90]),
        )

        self._side_cam = Camera(
            prim_path="/World/SceneCameras/SideCamera",
            position=np.array([-1.3, -8.9, 0.9]),
            orientation=to_quaternion([0, 0, 90]),
        )

        # Container prims

        containers = {
            "container_a": (0.5, "Violet"),
            "container_b": (0, "Blue"),
            "container_c": (-0.5, "Gray"),
        }

        for name, attributes in containers.items():
            y_pos = attributes[0]
            color = attributes[1]
            xform = self._import_obj(
                usd_path=f"{ext_assets_path()}/{container_usd_path}",
                prim_path=f"/World/Environment/Workspace/{name.title().replace('_', '')}",  # aka. ContainerA
                name=name,
                position=[-2.2, y_pos, 0.8],
                orientation_deg=[0, 0, 0],
                scale=[0.01, 0.01, 0.01],
            )

            color_variant = xform.prim.GetVariantSets().GetVariantSet("Color")
            color_variant.SetVariantSelection(color)

    def post_reset(self):
        pass
