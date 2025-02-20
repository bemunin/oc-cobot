import omni.ext

from .conveyor_room_sim import ConveyorRoomSim


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._sim = ConveyorRoomSim()
        self._sim.load(start=True, new_stage=False)

    def on_shutdown(self):
        self._sim = None
