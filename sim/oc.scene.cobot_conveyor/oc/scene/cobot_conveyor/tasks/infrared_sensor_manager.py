import carb
import numpy as np
import omni
import omni.graph.core as og
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.prims import (
    delete_prim,
    is_prim_path_valid,
    set_prim_attribute_value,
)
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import _sensor
from pxr import Gf


class InfraredSensorManager(BaseTask):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        # config vars
        self._sensor = _sensor.acquire_lightbeam_sensor_interface()
        self._debug = True
        self._enable = True

        # prim
        base_prim_path = "/World/Environment/Workspace/Sensors/InfraredSensor_Emitter"
        self._light_beam_path = f"{base_prim_path}/LightBeam"
        self._debug_og_path = f"{base_prim_path}/DebugActionGraph"

        if is_prim_path_valid(self._debug_og_path):
            delete_prim(self._debug_og_path)
            carb.log_info(
                "InfraredSensorManager: remove exisiting debug actiongraph prims"
            )

        # setup light beam sensor
        oriant_quat = euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)
        result, _ = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path=self._light_beam_path,
            parent=None,
            min_range=0.03,
            max_range=0.892,
            translation=Gf.Vec3d(0, -0.47581 * 2, 0),
            orientation=Gf.Quatd(
                oriant_quat[0], oriant_quat[1], oriant_quat[2], oriant_quat[3]
            ),
            forward_axis=Gf.Vec3d(0, 1, 0),
            num_rays=1,
            curtain_length=0.1,
        )

        if not result:
            carb.log_error("InfraredSensorManager: Could not create Light Beam Sensor")
            return

        self._create_debug_lightbeam_actiongraph()

    def post_reset(self):
        self.show_debug_beam(self._debug)

    def pre_step(self, time_step_index, simulation_time):
        if not self._enable:
            return

        is_detected, _, _ = self.sense()
        carb.log_info(f"InfraredSensorManager: is_detected={is_detected}")

    # helper functions
    def _create_debug_lightbeam_actiongraph(self):
        og.Controller.edit(
            {
                "graph_path": f"{self._debug_og_path}",
                "evaluator_name": "execution",
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadLightBeam", "omni.isaac.sensor.IsaacReadLightBeam"),
                    ("DebugDrawRayCast", "omni.isaac.debug_draw.DebugDrawRayCast"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (
                        "IsaacReadLightBeam.inputs:lightbeamPrim",
                        self._light_beam_path,
                    ),
                    ("DebugDrawRayCast.inputs:color", Gf.Vec4f(0.75, 0.75, 1.0, 0)),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "IsaacReadLightBeam.inputs:execIn"),
                    (
                        "IsaacReadLightBeam.outputs:execOut",
                        "DebugDrawRayCast.inputs:exec",
                    ),
                    (
                        "IsaacReadLightBeam.outputs:beamOrigins",
                        "DebugDrawRayCast.inputs:beamOrigins",
                    ),
                    (
                        "IsaacReadLightBeam.outputs:beamEndPoints",
                        "DebugDrawRayCast.inputs:beamEndPoints",
                    ),
                    (
                        "IsaacReadLightBeam.outputs:numRays",
                        "DebugDrawRayCast.inputs:numRays",
                    ),
                ],
            },
        )

    def show_debug_beam(self, is_show: bool):
        if is_show:
            self._debug = True
            set_prim_attribute_value(
                f"{self._debug_og_path}/DebugDrawRayCast",
                "inputs:color",
                Gf.Vec4f(0.75, 0.75, 1.0, 1.0),
            )
        else:
            self._debug = False
            set_prim_attribute_value(
                f"{self._debug_og_path}/DebugDrawRayCast",
                "inputs:color",
                Gf.Vec4f(0.75, 0.75, 1.0, 0),
            )

    def sense(self):
        """
        Determines the presence of an object using a light beam sensor and returns relevant data.
        It also returns the depth in meters and the hit position of the object.

        Returns:
            Tuple[bool, float, float]:
                - A boolean value indicating whether an object is detected (`True`) or not (`False`).
                - A float representing the depth of the detected object in meters.
                - A float representing the hit position of the detected object along forward axis set in IsaacSensorCreateLightBeamSensor.
        """
        is_detected = self._sensor.get_beam_hit_data(self._light_beam_path).astype(bool)
        depth_meter = self._sensor.get_linear_depth_data(self._light_beam_path)
        hit_pos = self._sensor.get_hit_pos_data(self._light_beam_path)
        return (is_detected, depth_meter, hit_pos)
