import numpy as np
from oc.utils.cobot.bases.base_sim import BaseSim
from oc.utils.cobot.robots.franka_manager import FrankaManager

from .tasks.env_creator import EnvCreator


class ConveyorRoomSim(BaseSim):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._world.add_task(EnvCreator(name="env_creator"))

        robot_pos = np.array([-1.5, 0, 0.8])
        self._world.add_task(FrankaManager(name="franka", offset=robot_pos))

    def post_reset(self):
        return
