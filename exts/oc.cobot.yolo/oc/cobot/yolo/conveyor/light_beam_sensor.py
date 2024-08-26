from typing import Tuple

import omni
import omni.graph.core as og
import omni.kit.commands
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.prims import (
    delete_prim,
    get_prim_at_path,
    set_prim_attribute_value,
)
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import _sensor
from pxr import Gf

import utils.log as log


class LightBeamSensor(XFormPrim):
    def __init__(self, name: str, prim_path: str):
        super().__init__(name=name, prim_path=prim_path)
        self._lb_sensor = _sensor.acquire_lightbeam_sensor_interface()
        self._is_visualize_on = False

        # prim
        self._light_beam_path = f"{prim_path}/LightBeam"
        self._visualizer_actiongraph_path = "/Main/LightBeamVisualizeGraph"

        # setup
        self._setup_light_beam_sensor()

    ##
    # Properties
    ##
    @property
    def is_visualize_on(self) -> bool:
        return self._is_visualize_on

    ##
    # Public Methods
    ##
    def sense(self) -> Tuple[bool, float, float]:
        """
        Determines the presence of an object using a light beam sensor and returns relevant data.
        It also returns the depth in meters and the hit position of the object.

        Returns:
            Tuple[bool, float, float]:
                - A boolean value indicating whether an object is detected (`True`) or not (`False`).
                - A float representing the depth of the detected object in meters.
                - A float representing the hit position of the detected object along forward axis set in IsaacSensorCreateLightBeamSensor.
        """
        is_detected = self._lb_sensor.get_beam_hit_data(self._light_beam_path).astype(
            bool
        )
        depth_meter = self._lb_sensor.get_linear_depth_data(self._light_beam_path)
        hit_pos = self._lb_sensor.get_hit_pos_data(self._light_beam_path)
        return (is_detected, depth_meter, hit_pos)

    def show_beam_visualizer(self, is_visualize: bool):
        """
        Set the visualization of the light beam sensor.

        Args:
            is_visualize (bool): A boolean value to set the visualization of the light beam sensor.
        """
        if is_visualize:
            self._is_visualize_on = True
            set_prim_attribute_value(
                f"{self._visualizer_actiongraph_path}/DebugDrawRayCast",
                "inputs:color",
                Gf.Vec4f(0.75, 0.75, 1.0, 1.0),
            )
        else:
            self._is_visualize_on = False
            set_prim_attribute_value(
                f"{self._visualizer_actiongraph_path}/DebugDrawRayCast",
                "inputs:color",
                Gf.Vec4f(0.75, 0.75, 1.0, 0),
            )

    ##
    # Private Methods
    ##
    def _setup_light_beam_sensor(self):
        light_beam_prim = get_prim_at_path(self._light_beam_path)

        # if light beam already exists, return
        if light_beam_prim.IsValid():
            delete_prim(self._visualizer_actiongraph_path)
            delete_prim(self._light_beam_path)
            log.info("Light beam sensor already exists. Deleting and recreating.")

        # light beam oriantation
        roll_deg = 0
        pitch_deg = 90
        yaw_deg = 0
        oriant_quat = euler_angles_to_quat([roll_deg, pitch_deg, yaw_deg], degrees=True)

        result, _ = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path=self._light_beam_path,
            parent=None,
            min_range=0.03,
            max_range=0.892,
            translation=Gf.Vec3d(0.02, -0.476, 0.01),
            orientation=Gf.Quatd(
                oriant_quat[0], oriant_quat[1], oriant_quat[2], oriant_quat[3]
            ),
            forward_axis=Gf.Vec3d(0, 1, 0),
            num_rays=1,
            curtain_length=0.1,
        )

        if not result:
            log.error("Could not create Light Beam Sensor")
            return

        self._setup_light_beam_visualizer()

    def _setup_light_beam_visualizer(self):
        og.Controller.edit(
            {
                "graph_path": f"{self._visualizer_actiongraph_path}",
                "evaluator_name": "execution",
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadLightBeam", "omni.isaac.sensor.IsaacReadLightBeam"),
                    ("DebugDrawRayCast", "omni.isaac.debug_draw.DebugDrawRayCast"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("IsaacReadLightBeam.inputs:lightbeamPrim", self._light_beam_path),
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
