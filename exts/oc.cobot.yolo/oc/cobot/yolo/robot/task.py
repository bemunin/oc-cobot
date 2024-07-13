from typing import Optional

import numpy as np
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask


class RobotTask(BaseTask):
    name = "robot_task"

    def __init__(
        self, name: Optional[str] = None, offset: Optional[np.ndarray] = None
    ) -> None:
        super().__init__(RobotTask.name if name is None else name, offset)

    ##
    #   Setup
    ##
    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)

    ##
    #   Required override methods
    ##
    def calculate_metrics(self) -> dict:
        return {}

    def get_observations(self):
        obs = {}
        return obs

    def is_done(self) -> bool:
        return False

    def get_params(self) -> dict:
        return {}

    def set_params(self, *args, **kwargs) -> None:
        pass

    ##
    #   Event hooks
    ##

    def pre_step(self, total_sim_step: int, total_sim_time_sec: float):
        return

    def post_reset(self) -> None:
        return

    def cleanup(self) -> None:
        return

    ##
    #   Routines
    ##

    # def _my_routine(self):
    #     pass

    ##
    #   Utils
    ##

    # def _my_utility(self):
    #     pass
