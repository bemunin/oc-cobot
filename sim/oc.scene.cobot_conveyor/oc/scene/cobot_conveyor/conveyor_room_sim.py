from oc.utils.cobot.bases.base_sim import BaseSim

from .tasks.env_creator import EnvCreator


class ConveyorRoomSim(BaseSim):
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._world.add_task(EnvCreator(name="env_creator"))

    def post_reset(self):
        return
