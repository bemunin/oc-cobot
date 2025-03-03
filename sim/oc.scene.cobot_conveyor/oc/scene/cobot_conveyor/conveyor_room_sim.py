import carb
import numpy as np
import omni.isaac.core.utils.prims as prims_utils
from oc.utils.cobot.bases.base_sim import BaseSim
from oc.utils.cobot.robots.franka_manager import FrankaManager
from omni.timeline import TimelineEventType

from .tasks.conveyor_manager import ConveyorManager
from .tasks.env_creator import EnvCreator
from .tasks.infrared_sensor_manager import InfraredSensorManager
from .tasks.tray_creator import TrayCreator


class ConveyorRoomSim(BaseSim):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        self._world.add_task(EnvCreator(name="env_creator"))
        self._world.add_task(TrayCreator(name="tray_creator"))
        self._world.add_task(ConveyorManager(name="conveyor_manager"))
        self._world.add_task(InfraredSensorManager(name="infrared_sensor_manager"))

        robot_pos = np.array([-1.5, 0, 0.8])
        self._world.add_task(FrankaManager(name="franka", offset=robot_pos))

        self._world.add_timeline_callback(
            "oc.scene.cobot_conveyor.timeline", self._on_timeline_callback
        )

    def post_reset(self):
        return

    def _on_timeline_callback(self, event):
        if event.type == int(TimelineEventType.STOP):
            carb.log_info("ConveyorRoomSim: Timeline stopped")
            carb.log_info("ConveyorRoomSim: Reset Execute prim group")

            if prims_utils.get_prim_at_path("/World/Execute"):
                prims_utils.delete_prim("/World/Execute")
