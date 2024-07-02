import isaacsim  # noqa: F401
from omni.isaac.kit import SimulationApp


CONFIG = {"renderer": "RayTracedLighting", "headless": False}
sim_app = SimulationApp(CONFIG)


from launch import (  # noqa: E402
    after_setup_stage,
    before_setup_stage,
    clear_world,
    init_world,
    running_sim,
    setup_stage,
)


if __name__ == "__main__":
    init_world()
    before_setup_stage()
    setup_stage(sim_app)
    after_setup_stage()
    running_sim(sim_app)
    clear_world()
    sim_app.close()
