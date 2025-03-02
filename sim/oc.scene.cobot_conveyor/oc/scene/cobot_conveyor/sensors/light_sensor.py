import carb
import numpy as np
import omni
import omni.graph.core as og
from omni.isaac.core.utils.prims import (
    delete_prim,
    is_prim_path_valid,
    set_prim_attribute_value,
)
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import _sensor
from pxr import Gf


class LightSensor:
    LIGHTBEAM_PRIM_NAME = "LightBeam"
    DEBUG_PRIM_NAME = "DebugActionGraph"

    def __init__(self, prim_path: str, debug_mode: bool = False):
        self._debug = debug_mode
        self._lightbeam_path = f"{prim_path}/{LightSensor.LIGHTBEAM_PRIM_NAME}"
        self._debug_og_path = f"{prim_path}/{LightSensor.DEBUG_PRIM_NAME}"

        self._sensor = _sensor.acquire_lightbeam_sensor_interface()

        if is_prim_path_valid(self._debug_og_path):
            delete_prim(self._debug_og_path)
            carb.log_info("LightSensor: remove exisiting debug actiongraph prims")

        result = self._create_lightbeam_prim()

        if result:
            self._create_debug_actiongraph()
            self.set_debug_mode(debug_mode)

    # APIs
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
        is_detected = self._sensor.get_beam_hit_data(self._lightbeam_path).astype(bool)
        depth_meter = self._sensor.get_linear_depth_data(self._lightbeam_path)
        hit_pos = self._sensor.get_hit_pos_data(self._lightbeam_path)
        return (is_detected, depth_meter, hit_pos)

    def set_debug_mode(self, enable: bool):
        self._debug = enable
        color = (
            Gf.Vec4f(0.75, 0.75, 1.0, 1.0) if enable else Gf.Vec4f(0.75, 0.75, 1.0, 0)
        )

        set_prim_attribute_value(
            f"{self._debug_og_path}/DebugDrawRayCast", "inputs:color", color
        )

    # Helper functions
    def _create_lightbeam_prim(self):
        oriant_quat = euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)
        result, _ = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path=self._lightbeam_path,
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
            carb.log_error("LightSensor: Could not create LightBeam child prim")

        return result

    def _create_debug_actiongraph(self):
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
                        self._lightbeam_path,
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
