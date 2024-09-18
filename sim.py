# ruff: noqa: E402
import isaacsim  # noqa: F401
from omni.isaac.kit import SimulationApp


CONFIG = {"renderer": "RayTracedLighting", "headless": False}
sim_app = SimulationApp(CONFIG)

import os
import sys

import omni
from omni.isaac.core import World
from omni.isaac.core.utils import extensions


def init_world():
    world_settings = {
        "physics_dt": 1.0 / 60.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
    }
    World(**world_settings)

    world = World.instance()
    world.initialize_physics()


def register_extensions(ext_scene: str):
    # add custom extension path
    manager = omni.kit.app.get_app().get_extension_manager()
    oc_cobot_exts_path = f"{os.path.dirname(__file__)}/exts"
    manager.add_path(oc_cobot_exts_path)

    ext_scene = ext_scene if ext_scene else "conveyor"

    # enable required extensions
    extensions.enable_extension("omni.kit.tool.measure")
    extensions.enable_extension("omni.isaac.conveyor")
    extensions.enable_extension("omni.isaac.ros2_bridge")
    extensions.enable_extension(f"oc.cobot.{ext_scene}")


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


if __name__ == "__main__":
    # get omnicraft cobot extension name from command line arguments
    args = sys.argv[1:]
    oc_cobot_ext_scene = args[0] if args else None

    init_world()
    register_extensions(oc_cobot_ext_scene)
    running_sim(sim_app)
    clear_world()
    sim_app.close()
