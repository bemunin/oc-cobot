import gc

import omni


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id

    def on_shutdown(self):
        gc.collect()
