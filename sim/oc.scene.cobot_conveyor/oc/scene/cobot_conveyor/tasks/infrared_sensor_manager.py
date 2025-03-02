import carb
from omni.isaac.core import World
from omni.isaac.core.tasks import BaseTask

from ..sensors import LightSensor


class InfraredSensorManager(BaseTask):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        # config vars

        self._enable = True

        # prim
        sensor_prim_path = "/World/Environment/Workspace/Sensors/InfraredSensor_Emitter"
        self._sensor = LightSensor(sensor_prim_path, debug_mode=False)
        self._conveyor = None

    def post_reset(self):
        self._conveyor = World().instance().get_task("conveyor_manager")

    def pre_step(self, time_step_index, simulation_time):
        if not self._enable:
            return

        is_detected, _, _ = self._sensor.sense()

        try:
            if is_detected:
                self._conveyor.stop()
            else:
                self._conveyor.start()
        except Exception as e:
            carb.log_error(f"InfraredSensorManager: {e}")
