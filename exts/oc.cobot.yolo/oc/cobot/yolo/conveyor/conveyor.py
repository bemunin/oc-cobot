import numpy as np
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.prims import set_prim_attribute_value
from omni.isaac.core.world.world import World
from pxr import Gf

import utils.log as log

from .light_beam_sensor import LightBeamSensor


class Conveyor(XFormPrim):
    SENSOR_POS_ADJUST_UNIT = 0.1  # 1 unit = 0.1m

    def __init__(self, name: str, prim_path: str):
        super().__init__(name=name, prim_path=prim_path)

        # vars
        self._prim_path = prim_path

        # status
        self._is_engine_on = True
        self._is_belt_move = False
        self._is_sensor_on = True

        # configs
        self._speed_m_per_sec = -0.2
        self._sensor_default_x_pos = -1.1
        self._sensor_pos = None  # np.array 3d

        # belt
        self._start_pos = None
        self._end_pos = None

        # sensor
        self._sensor = LightBeamSensor(
            name="LightBeamSensor", prim_path=f"{prim_path}/Sensors"
        )

        # callback
        world = World.instance()
        world.add_physics_callback("conveyor:sim_step", self._on_sim_step)

    ##
    # Properties
    ##
    @property
    def is_belt_move(self) -> bool:
        return self._is_belt_move

    @property
    def is_engine_on(self) -> bool:
        return self._is_engine_on

    @property
    def is_sensor_on(self) -> bool:
        return self._is_sensor_on

    @property
    def sensor_pos(self) -> np.array:
        if self._sensor_pos is not None:
            return self._sensor_pos

        sensor_pos, _ = self._sensor.get_local_pose()
        self._sensor_pos = sensor_pos
        return self._sensor_pos

    @property
    def start_pos(self) -> np.array:
        if self._start_pos is not None:
            return self._start_pos

        belt = XFormPrim(f"{self.prim_path}/Track/ConveyorTrackA/Belt")
        belt_pos, _ = belt.get_world_pose()
        conveyor_scale = self.get_world_scale()
        belt_height = belt_pos[2] * conveyor_scale[2]  # z-axis
        belt_center = belt_pos[1] * conveyor_scale[1]  # y-axis

        pole = XFormPrim(
            f"{self.prim_path}/Track/ConveyorTrackA/SM_ConveyorBelt_A06_Decal_02"
        )
        pole_pos, _ = pole.get_world_pose()
        pole_pos_x = pole_pos[0]
        self._start_pos = np.array([pole_pos_x, belt_center, belt_height])
        return self._start_pos

    @property
    def end_pos(self) -> np.array:
        if self._end_pos is not None:
            return self._end_pos

        belt = XFormPrim(f"{self.prim_path}/Track/ConveyorTrackB/Belt")
        belt_pos, _ = belt.get_world_pose()
        conveyor_scale = self.get_world_scale()
        belt_height = belt_pos[2] * conveyor_scale[2]  # z-axis
        belt_center = belt_pos[1] * conveyor_scale[1]  # y-axis

        pole = XFormPrim(
            f"{self.prim_path}/Track/ConveyorTrackB/SM_ConveyorBelt_A06_Decal_02"
        )
        pole_pos, _ = pole.get_world_pose()
        pole_pos_x = pole_pos[0]
        self._end_pos = np.array([pole_pos_x, belt_center, belt_height])
        return self._end_pos

    ##
    # Engine Control API
    ##

    def start(self):
        if self._is_belt_move:
            return

        self.set_prim_vel("ConveyorTrackA", self._speed_m_per_sec)
        self.set_prim_vel("ConveyorTrackB", self._speed_m_per_sec)

        self._is_belt_move = True

    def stop(self):
        if not self._is_belt_move:
            return

        self.set_prim_vel("ConveyorTrackA", 0.0)
        self.set_prim_vel("ConveyorTrackB", 0.0)

        self._is_belt_move = False

    def turn_on(self):
        self._is_engine_on = True
        self.start()

    def turn_off(self):
        self._is_engine_on = False
        self.stop()

    ##
    # Sensor API
    ##

    def enable_sensor(self):
        self._is_sensor_on = True

    def disable_sensor(self):
        self._is_sensor_on = False

    def move_sensor(self, distance: int):
        """move sensor forward or backward in x-axis from its default position

        Args:
            distance (int): distance to move in unit step. 1 unit step = 0.1m (negative: forward, position: backward)
        """

        new_pos_x = (
            self._sensor_default_x_pos + distance * Conveyor.SENSOR_POS_ADJUST_UNIT
        )

        sensor_pos, _ = self._sensor.get_local_pose()
        new_pos = np.array([new_pos_x, sensor_pos[1], sensor_pos[2]])

        self._sensor.set_local_pose(new_pos)
        self._sensor_pos = new_pos

    def get_sensor_move_distance(self) -> int:
        """get distance changed in x-axis of sensor compared to its default position

        Returns:
            int: moved distance in unit step. 1 unit step = 0.1m (negative: forward, position: backward)
        """
        pos = self.sensor_pos
        cur_pos_x = round(pos[0], 1)
        delta = round(
            (cur_pos_x - self._sensor_default_x_pos) / Conveyor.SENSOR_POS_ADJUST_UNIT
        )
        return delta

    def is_sensor_detect_object(self) -> bool:
        if not self._is_sensor_on:
            return False

        is_detected, _, _ = self._sensor.sense()

        return is_detected

    def get_sensor_prim(self) -> LightBeamSensor:
        return self._sensor

    ##
    # Callback
    ##
    def _on_sim_step(self, time_per_step_ms: float):
        try:
            if not self._is_engine_on:
                return
            is_detect = self.is_sensor_detect_object()
            if is_detect:
                return self.stop()
            else:
                return self.start()
        except Exception as e:
            log.error(f"Error on sensor detection: {e}")
            return

    ##
    # Utils
    ##

    def set_prim_vel(self, track: str, vel: float):
        set_prim_attribute_value(
            f"{self._prim_path}/Track/{track}/ConveyorBeltGraph/ConveyorNode",
            "inputs:velocity",
            vel,
        )

    def set_prim_direction(self, track: str, x_direction: float):
        """set direction input to conveyor prim action graph

        Args:
            track (str): track prim name e.g. ConveyorTrackA
            x_direction (float): direction in x-axis (1.0 or -1.0)
        """
        set_prim_attribute_value(
            f"{self._prim_path}/Track/{track}/ConveyorBeltGraph/ConveyorNode",
            "inputs:direction",
            Gf.Vec3f(x_direction, 0, 0),
        )
