import isaacsim
from omni.isaac.kit import SimulationApp


CONFIG = {"renderer": "RayTracedLighting", "headless": False}
sim_app = SimulationApp(CONFIG)

from launch import create_world, run  # noqa: E402


if __name__ == "__main__":
    create_world()
    run(sim_app)
    sim_app.close()
