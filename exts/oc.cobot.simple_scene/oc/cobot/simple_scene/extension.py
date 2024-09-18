import gc

import omni

import utils.log as log


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        log.info("simple_scene: Simple scene extension is loaded.")

    def on_shutdown(self):
        gc.collect()
