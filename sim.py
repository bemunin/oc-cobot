import isaacsim  # noqa: F401
from omni.isaac.kit import SimulationApp


CONFIG = {"renderer": "RayTracedLighting", "headless": False}
sim_app = SimulationApp(CONFIG)


from launch import (  # noqa: E402
    clear_world,
    init_world,
    register_extensions,
    setup_stage,
)


if __name__ == "__main__":
    world = init_world()
    register_extensions()
    setup_stage(sim_app)

    world.play()

    while sim_app.is_running():
        world.step()

    clear_world()
    sim_app.close()
