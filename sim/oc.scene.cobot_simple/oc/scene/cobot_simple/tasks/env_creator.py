import numpy as np
from oc.utils.cobot.bases.creator_task import CreatorTask
from omni.isaac.core.objects import DynamicCuboid, DynamicCylinder
from omni.isaac.core.scenes import Scene


class EnvCreator(CreatorTask):
    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()

        # object position
        pos_x = 0.6
        pos_y = 0.15

        # cylinder
        height_m = 0.1
        radius_m = 0.02
        mass_kg = 0.1  # kg
        scene.add(
            DynamicCylinder(
                prim_path="/World/Cylinder",
                name="cylinder",
                position=np.array([pos_x, pos_y, height_m / 2.0]),
                radius=radius_m,
                height=height_m,
                mass=mass_kg,
            )
        )

        # cube
        size_m = 0.06
        mass_kg = 0.1
        scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="cube",
                position=np.array([pos_x, -pos_y, size_m / 2.0]),
                size=size_m,
                mass=mass_kg,
            )
        )

    def post_reset(self):
        return
