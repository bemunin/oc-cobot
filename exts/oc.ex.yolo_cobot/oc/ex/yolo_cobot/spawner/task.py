import random
from collections import deque

import numpy as np
import toml
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.stage import clear_stage
from omni.isaac.core.world.world import World

from utils.path import get_proj_root_path

from ..conveyor import Conveyor
from .enums import ObjectItem, ObjectType
from .helpers import random_object, spawn


class SpawnerTask(BaseTask):
    name = "spawner_task"

    def __init__(self):
        super().__init__(SpawnerTask.name)

        root = get_proj_root_path()

        try:
            with open(f"{root}/config.dev.toml", "r") as file:
                config = toml.load(file)
        except FileNotFoundError:
            with open(f"{root}/config.toml", "r") as file:
                config = toml.load(file)

        # variables
        self._items_on_belt = deque()
        self._items_off_belt = deque()
        self._total_items = 0
        # step, need to use it for uniform spawn
        # in case of conveyor start/stop
        self._spawn_timer_count = 0

        # configs
        self._disabled_task = False
        # 90 step = every 2.5 sec (60Hz sim step)
        self._spawn_timer = 180  # step
        self._object_type = (
            ObjectType.REAL_LIFE
            if config["spawner"]["object_type"] == "real_life"
            else ObjectType.GEOMETRIC
        )
        # ObjectItem.ALL will random all object items filter by the object_type
        self._object_spawn = (
            ObjectItem.ALL
            if config["spawner"]["object_spawn"] == "all"
            else ObjectItem(config["spawner"]["object_spawn"])
        )
        self._is_random_angle = False
        self._is_random_pos = False

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
    # Task
    ##

    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)

    def get_observations(self):
        obs = {
            "items_on_belt": self._items_on_belt,
            "items_off_belt": self._items_off_belt,
            "total_items": self._total_items,
        }
        return obs

    def pre_step(self, total_sim_step: int, total_sim_time_sec: float):
        if self._disabled_task:
            return

        kwargs = {
            "total_sim_step": total_sim_step,
            "total_sim_time_sec": total_sim_time_sec,
        }

        self._spawn_routine(**kwargs)
        self._manage_items_on_belt_routine(**kwargs)

    def cleanup(self) -> None:
        """
        clean up temporary objects during task execution,
        use when reset or shutdown task
        """
        self._items_on_belt = deque()
        self._items_off_belt = deque()
        self._total_items = 0
        self._spawn_timer_count = 0
        clear_stage(lambda path: path == "/Execute/spawner_task")

    ##
    # Routines
    ##
    def _spawn_routine(self, total_sim_step: int, total_sim_time_sec: float):
        # don't spawn if belt is stopped
        if not self.conveyor.is_belt_move:
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
            self._items_on_belt.append(item)
            self._total_items += 1

    def _manage_items_on_belt_routine(
        self, total_sim_step: int, total_sim_time_sec: float
    ):
        """Manage first item on belt or off belt

        Args:
            time_step_index (_type_): _description_
            simulation_time (_type_): _description_
        """

        # checkout everyr 5 time steps (10ms)

        # Check and remove if first item is not on the belt
        if not self._items_on_belt:
            return

        first_item = self._items_on_belt[0]
        first_item_pose, _ = first_item.get_local_pose()
        first_item_x = first_item_pose[0]
        first_item_y = first_item_pose[1]
        first_item_z = first_item_pose[2]

        # check if the first item y pos is still on the belt
        if first_item_y > 0.48 or first_item_y < -0.48:
            self._move_first_item_off_belt()
            return

        delta_z = abs(first_item_z - 0.58)

        # check if the first item z pos is still on the belt
        if delta_z > 0.5:
            self._move_first_item_off_belt()
            return

        # pop first item if it's passed the sensor
        sensor_x = self.conveyor.sensor_pos[0]
        if first_item_x > sensor_x + 0.01:
            self._move_first_item_off_belt()
            return

    ##
    # Utils
    ##
    def _move_first_item_off_belt(self):
        if not self._items_on_belt:
            return

        first_item = self._items_on_belt.popleft()
        self._items_off_belt.append(first_item)

    def _get_spawn_point(self) -> np.ndarray:
        if self._spawn_point is None:
            scene = World.instance().scene
            conveyor = scene.get_object("conveyor")

            spawn_height_offset = [0, 0, 0.15]
            spawn_end_offset = [0.2, 0, 0]

            spawn_point = conveyor.start_pos + spawn_height_offset + spawn_end_offset
            # cache
            self._spawn_point = spawn_point

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
