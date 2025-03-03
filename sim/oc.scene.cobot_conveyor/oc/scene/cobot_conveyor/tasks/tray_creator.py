import carb
from oc.utils.cobot.bases.creator_task import CreatorTask
from omni.isaac.core import World
from omni.isaac.robot_description_editor import XFormPrim
from omni.physx.scripts import utils
from pxr import UsdPhysics

from ..utils import ext_assets_path


class Timer:
    def __init__(self, period_sec):
        self._period_sec = period_sec
        self._time_step = 0

        self._time_per_step_sec = World.instance().get_physics_dt()

    def tick(self):
        self._time_step += 1

        if self._time_step * self._time_per_step_sec >= self._period_sec:
            self._time_step = 0
            return True

        return False


class TrayCreator(CreatorTask):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        # configs

        # variables
        self._spawn_period_sec = 2.5
        self._timer = Timer(self._spawn_period_sec)
        self._tray_count = 0
        self._conveyor = None

    def post_reset(self):
        super().post_reset()
        self._spawn_tray()
        self._spawn_objects()
        self._conveyor = World.instance().get_task("conveyor_manager")

    def pre_step(self, time_step_index, simulation_time):
        super().pre_step(time_step_index, simulation_time)
        timer_triggered = False

        if self._conveyor.status == "start":
            timer_triggered = self._timer.tick()

        if timer_triggered:
            carb.log_info("TrayCreator: spawn new tray")
            self._spawn_tray()
            self._spawn_objects()

    def _spawn_tray(self):
        self._tray_count += 1
        container_usd_path = "Collected_Container_C19_61x40x18cm_PR_V_NVD_01/Container_C19_61x40x18cm_PR_V_NVD_01.usd"

        xform = self._import_obj(
            usd_path=f"{ext_assets_path()}/{container_usd_path}",
            prim_path=f"/World/Execute/Tray{self._tray_count:03}",  # aka. ContainerA
            name=f"tray{self._tray_count:03}",
            position=[1.1, 0, 0.8],
            orientation_deg=[0, 0, 90],
            scale=[0.01, 0.01, 0.01],
            prim_type="XFormPrim",
        )
        # set object collision
        utils.setRigidBody(xform.prim, "convexDecomposition", False)

    def _spawn_objects(self):
        current_tray = XFormPrim(f"/World/Execute/Tray{self._tray_count:03}")
        tray_pos, _ = current_tray.get_world_pose()
        obj_pos = tray_pos + [0, 0, 0.02]
        # spawn objects
        xform_obj = self._import_obj(
            usd_path=f"{ext_assets_path()}/objects/cube.usda",
            prim_path=f"/World/Execute/ObjectSet{self._tray_count:03}/Cube01",
            name="cube01",
            position=obj_pos,
            orientation_deg=[0, 0, 0],
            scale=[1, 1, 1],
            prim_type="XForm",
        )
        utils.setRigidBody(xform_obj.prim, "convexHull", False)
        mass_api = UsdPhysics.MassAPI.Apply(xform_obj.prim)
        mass_api.CreateMassAttr(0.1)  # 0.1kg
