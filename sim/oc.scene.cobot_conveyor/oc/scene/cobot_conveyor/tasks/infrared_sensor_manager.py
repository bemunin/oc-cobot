import carb
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

    def post_reset(self):
        return

    def pre_step(self, time_step_index, simulation_time):
        if not self._enable:
            return

        is_detected, _, _ = self._sensor.sense()
        carb.log_info(f"InfraredSensorManager: is_detected={is_detected}")
