import isaacsim  # noqa: F401
from omni.isaac.kit import SimulationApp


CONFIG = {"renderer": "RayTracedLighting", "headless": False}
sim_app = SimulationApp(CONFIG)


from launch import (  # noqa: E402
    clear_world,
    init_world,
    register_extensions,
    running_sim,
)


if __name__ == "__main__":
    init_world()
    register_extensions()
    running_sim(sim_app)
    clear_world()
    sim_app.close()
