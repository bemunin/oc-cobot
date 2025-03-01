import numpy as np
from oc.utils.cobot.bases.base_sim import BaseSim
from oc.utils.cobot.robots.franka_manager import FrankaManager

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

    def post_reset(self):
        return
