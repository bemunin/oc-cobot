import os

import isaacsim  # noqa: F401
import omni
from omni.isaac.core import World
from omni.isaac.core.utils import extensions
from omni.isaac.kit import SimulationApp


def init_world():
    world_settings = {
        "physics_dt": 1.0 / 60.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
    }
    World(**world_settings)

    world = World.instance()
    world.initialize_physics()


def register_extensions():
    # add custom extension path
    manager = omni.kit.app.get_app().get_extension_manager()
    oc_cobot_exts_path = f"{os.path.dirname(__file__)}/../exts"
    manager.add_path(oc_cobot_exts_path)

    # enable required extensions
    extensions.enable_extension("omni.kit.tool.measure")
    extensions.enable_extension("omni.isaac.conveyor")
    extensions.enable_extension("omni.isaac.ros2_bridge")
    extensions.enable_extension("oc.cobot.yolo")


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
