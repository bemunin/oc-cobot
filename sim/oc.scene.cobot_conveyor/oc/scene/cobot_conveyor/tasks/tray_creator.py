from oc.utils.cobot.bases.creator_task import CreatorTask
from omni.isaac.core import World
from omni.physx.scripts import utils

from ..utils import ext_assets_path


class TrayCreator(CreatorTask):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        # configs

        # variables
        self._timer = 0
        self._conveyor = None

        container_usd_path = "Collected_Container_C19_61x40x18cm_PR_V_NVD_01/Container_C19_61x40x18cm_PR_V_NVD_01.usd"
        rigid = self._import_obj(
            usd_path=f"{ext_assets_path()}/{container_usd_path}",
            prim_path="/World/Execute/Tray01",  # aka. ContainerA
            name="tray01",
            position=[1.1, 0, 0.8],
            orientation_deg=[0, 0, 90],
            scale=[0.01, 0.01, 0.01],
            prim_type="RigidPrim",
        )

        # set object collision
        utils.setRigidBody(rigid.prim, "convexDecomposition", False)
        scene.add(rigid)

    def post_reset(self):
        super().post_reset()
        self._conveyor = World.instance().get_task("conveyor_manager")

    def pre_step(self, time_step_index, simulation_time):
        super().pre_step(time_step_index, simulation_time)
        # carb.log_info(
        #     f"TrayCreator: Time step index: {time_step_index}, Simulation time: {simulation_time}"
        # )
