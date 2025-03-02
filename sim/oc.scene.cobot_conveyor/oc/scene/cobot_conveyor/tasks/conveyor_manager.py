from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import Gf, PhysxSchema


class ConveyorManager(BaseTask):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        # configs
        self._belt_speed = 0.25
        self._status = "start"  # start, stop, shutdown

        # usd prims
        # Reason to use usd prim instead of IsaacSim prim wrapper api such as XForm
        # is because IsaacSim prim wrapper will override physics api setting in base_env.usda
        # however retrive usd prim with get_prim_at_path is not override
        self._conveyor = get_prim_at_path(
            prim_path="/World/Environment/Workspace/ConveyorBelt_A08"
        )

        self._rollers = get_prim_at_path(
            "/World/Environment/Workspace/ConveyorBelt_A08/Rollers"
        )

        self._surface_api = None

    def post_reset(self):
        surface_linear_vel = Gf.Vec3f(-self._belt_speed, 0.0, 0.0)
        self._surface_api = PhysxSchema.PhysxSurfaceVelocityAPI(self._rollers)
        self._surface_api.GetSurfaceVelocityAttr().Set(surface_linear_vel)

    def pre_step(self, time_step_index, simulation_time):
        super().pre_step(time_step_index, simulation_time)

    # APIS
    def start(self):
        if self._status == "start":
            return

        self._status = "start"
        surface_linear_vel = Gf.Vec3f(-self._belt_speed, 0.0, 0.0)
        self._surface_api.GetSurfaceVelocityEnabledAttr().Set(True)
        self._surface_api.GetSurfaceVelocityAttr().Set(surface_linear_vel)

    def stop(self):
        if self._status == "stop":
            return

        self._status = "stop"
        surface_linear_vel = Gf.Vec3f(0.0, 0.0, 0.0)
        self._surface_api.GetSurfaceVelocityEnabledAttr().Set(False)
        self._surface_api.GetSurfaceVelocityAttr().Set(surface_linear_vel)
