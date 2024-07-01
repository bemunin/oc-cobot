import isaacsim
from omni.isaac.core import World
from omni.isaac.kit import SimulationApp


def create_world():
    world_settings = {
        "physics_dt": 1.0 / 60.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
    }
    World(**world_settings)


def run(sim_app: SimulationApp):
    world = World.instance()
    world.initialize_physics()
    world.play()

    while sim_app.is_running():
        world.step()

    world.stop()
    world.clear_all_callbacks()
    world.clear_instance()
