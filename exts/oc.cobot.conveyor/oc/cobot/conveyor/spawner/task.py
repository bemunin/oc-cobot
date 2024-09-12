import random
from collections import deque

import numpy as np
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.world.world import World

from utils.path import load_config

from ..conveyor import Conveyor
from .enums import ObjectItem, ObjectType
from .helpers import random_object, spawn


class SpawnerTask(BaseTask):
    name = "spawner_task"

    def __init__(self):
        super().__init__(SpawnerTask.name)
        config = load_config()

        # state control
        self._is_task_run = True

        # variables
        self._items = deque()
        self._total_items = 0
        # step, need to use it for uniform spawn
        # in case of conveyor start/stop
        self._spawn_timer_count = 0

        # configs
        self._disabled_task = False
        # 180 step = every 3 sec (60Hz sim step)
        self._spawn_timer = 180  # step

        config_object_type = config.get("spawner", {}).get("object_type", "real_life")
        self._object_type = (
            ObjectType.REAL_LIFE
            if config_object_type == "real_life"
            else ObjectType.GEOMETRIC
        )

        # ObjectItem.ALL will random all object items filter by the object_type
        config_object_spawn = config.get("spawner", {}).get("object_spawn", "all")
        self._object_spawn = (
            ObjectItem.ALL
            if config_object_spawn == "all"
            else ObjectItem(config_object_spawn)
        )
        self._is_random_angle = False
        self._is_random_pos = False

        # Adjust spawn point

        self._spawn_pos_x_offset = config.get("conveyor", {}).get(
            "spawn_pos_x_offset", 0.0
        )

        # cache
        self._spawn_point = None
        self._conveyor = None
        self._object_usd_table = {}

    ##
    # Properties
    ##
    @property
    def conveyor(self) -> Conveyor:
        if self._conveyor is not None:
            return self._conveyor

        scene = World.instance().scene
        conveyor = scene.get_object("conveyor")
        return conveyor

    @property
    def object_type(self) -> ObjectType:
        return self._object_type

    @object_type.setter
    def object_type(self, object_type: ObjectType):
        self._object_type = object_type

    @property
    def object_spawn(self) -> ObjectItem:
        return self._object_spawn

    @object_spawn.setter
    def object_spawn(self, object_spawn: ObjectItem):
        self._object_spawn = object_spawn

    @property
    def is_random_angle(self) -> bool:
        return self._is_random_angle

    @property
    def is_task_run(self) -> bool:
        return self._is_task_run

    @is_random_angle.setter
    def is_random_angle(self, is_random_angle: bool):
        self._is_random_angle = is_random_angle

    @property
    def is_random_pos(self) -> bool:
        return self._is_random_pos

    @is_random_pos.setter
    def is_random_pos(self, is_random_pos: bool):
        self._is_random_pos = is_random_pos

    ##
    # API
    ##
    def run(self):
        self._is_task_run = True

    def stop(self):
        self._is_task_run = False

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
        obs = {
            "items": self._items,
            "total_items": self._total_items,
        }
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
        if self._disabled_task:
            return

        kwargs = {
            "total_sim_step": total_sim_step,
            "total_sim_time_sec": total_sim_time_sec,
        }

        self._spawn_routine(**kwargs)

    def post_reset(self) -> None:
        return

    def cleanup(self) -> None:
        """
        clean up temporary objects during task execution,
        use when reset or shutdown task
        """
        self._items = deque()
        self._total_items = 0
        self._spawn_timer_count = 0

        if prims_utils.get_prim_at_path("/Execute/spawner_task"):
            prims_utils.delete_prim("/Execute/spawner_task")

    ##
    # Routines
    ##
    def _spawn_routine(self, total_sim_step: int, total_sim_time_sec: float):
        # don't spawn if belt is stopped
        if not self.conveyor.is_belt_move or not self._is_task_run:
            return

        # check if spawn timer trigger
        if not self._is_spawn_timer_trigger(total_sim_step):
            return

        if self._object_spawn == ObjectItem.ALL:
            spawn_object = random_object(self._object_type)
        else:
            spawn_object = self._object_spawn

        spawn_kwargs = {
            "task_name": SpawnerTask.name,
            "spawn_point": self._get_spawn_point(),
            "object_item": spawn_object,
            "total_items": self._total_items,
            "orient_deg": self._get_spawn_oriant_deg(),
        }

        item = spawn(**spawn_kwargs)

        if item is not None:
            self._items.append(item)
            self._total_items += 1

    ##
    # Utils
    ##

    def _get_spawn_point(self) -> np.ndarray:
        if self._spawn_point is None:
            scene = World.instance().scene
            conveyor = scene.get_object("conveyor")

            spawn_height_offset = [0, 0, 0.15]
            spawn_end_offset = [0.2, 0, 0]

            spawn_point = conveyor.start_pos + spawn_height_offset + spawn_end_offset
            # cache
            self._spawn_point = spawn_point

            # adjust starting point to first part of the conveyor
            self._spawn_point[0] = self._spawn_point[0] + self._spawn_pos_x_offset

        if self._is_random_pos:
            spawn_point = self._spawn_point.copy()
            # random only in y axis (side to side)
            spawn_point[1] = random.uniform(-0.24, 0.24)
            return spawn_point
        else:
            return self._spawn_point

    def _get_spawn_oriant_deg(self) -> np.ndarray:
        # object pose
        yaw_deg = 0
        pitch_deg = 0
        roll_deg = 0

        if self._is_random_angle:
            roll_deg = random.uniform(0, 359)

        orient_deg = np.array([yaw_deg, pitch_deg, roll_deg])
        return orient_deg

    def _is_spawn_timer_trigger(self, total_sim_step: int):
        if not self.conveyor.is_belt_move:
            return

        if self._spawn_timer_count >= self._spawn_timer:
            self._spawn_timer_count = 0
            return True

        self._spawn_timer_count += 1
        return False
