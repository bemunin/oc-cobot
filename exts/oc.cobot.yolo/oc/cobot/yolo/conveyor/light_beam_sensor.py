import omni
import omni.graph.core as og
import omni.kit.commands
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.utils.prims import delete_prim, get_prim_at_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import _sensor
from pxr import Gf

import utils.log as log


class LightBeamSensor(XFormPrim):
    def __init__(self, name: str, prim_path: str):
        super().__init__(name=name, prim_path=prim_path)
        self._lb_sensor = _sensor.acquire_lightbeam_sensor_interface()

        # prim
        self._light_beam_path = f"{prim_path}/LightBeam"
        self._visualizer_actiongraph_path = "/Main/LightBeamVisualizeGraph"

        # setup
        self._setup_light_beam_sensor()

    ##
    # Public Methods
    ##

    def enable_sensor(self):
        pass

    def disable_sensor(self):
        pass

    def move_sensor(self, distance: int):
        pass

    def get_sensor_move_distance(self) -> int:
        pass

    def is_sensor_detect_object(self) -> bool:
        pass

    ##
    # Private Methods
    ##
    def _setup_light_beam_sensor(self):
        light_beam_prim = get_prim_at_path(self._light_beam_path)

        # if light beam already exists, return
        if light_beam_prim.IsValid():
            delete_prim(self._light_beam_path)
            delete_prim(self._visualizer_actiongraph_path)
            log.info("Light beam sensor already exists. Deleting and recreating.")

        # light beam oriantation
        roll_deg = 0
        pitch_deg = 90
        yaw_deg = 0
        oriant_quat = euler_angles_to_quat([roll_deg, pitch_deg, yaw_deg], degrees=True)
        # calculate light beam max range

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path=self._light_beam_path,
            parent=None,
            min_range=0.03,
            max_range=0.892,
            translation=Gf.Vec3d(0, -0.476, 0.01),
            orientation=Gf.Quatd(
                oriant_quat[0], oriant_quat[1], oriant_quat[2], oriant_quat[3]
            ),
            forward_axis=Gf.Vec3d(0, 1, 0),
            num_rays=1,
            curtain_length=2,
        )

        if not result:
            log.error("Could not create Light Beam Sensor")
            return

        self._visualize_light_beam()

    def _visualize_light_beam(self):
        (action_graph, new_nodes, _, _) = og.Controller.edit(
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
