import os

import isaacsim  # noqa: F401
import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.utils import extensions, stage, viewports
from omni.isaac.kit import SimulationApp

from utils.path import get_proj_root_path


def init_world():
    world_settings = {
        "physics_dt": 1.0 / 60.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
    }
    World(**world_settings)

    world = World.instance()
    world.initialize_physics()


def before_setup_stage():
    # enable required extensions
    extensions.enable_extension("omni.isaac.conveyor")


def setup_stage(sim_app: SimulationApp):
    # camera view setup
    camera_pos = np.array([-4, -4, 4])
    camera_angle_point = np.array([-1, 1, 0])
    viewports.set_camera_view(eye=camera_pos, target=camera_angle_point)

    env_usd_path = get_proj_root_path() + "/assets/environments/default.usda"
    workcell_usd_path = get_proj_root_path() + "/assets/workcell.usda"

    stage.add_reference_to_stage(env_usd_path, "/Environment")
    stage.add_reference_to_stage(workcell_usd_path, "/Main")

    sim_app.update()


def after_setup_stage():
    # add custom extension path
    manager = omni.kit.app.get_app().get_extension_manager()
    yolo_cobot_exts_path = f"{os.path.dirname(__file__)}/../exts"
    manager.add_path(yolo_cobot_exts_path)

    extensions.enable_extension("oc.ex.yolo_cobot")


def running_sim(sim_app: SimulationApp):
    world = World.instance()
    world.play()
    while sim_app.is_running():
        world.step()


def clear_world():
    world = World.instance()
    world.stop()
    world.clear_all_callbacks()
    world.clear_instance()
