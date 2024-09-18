import gc
import weakref

import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import stage
from omni.isaac.franka import Franka
from omni.isaac.ui.menu import make_menu_item_description
from omni.kit.menu.utils.scripts.utils import add_menu_items, remove_menu_items

from utils import get_proj_root_path, log


EXTENSION_WINDOW_NAME = "Simple Scene Panel"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id

        # UIs & Menus
        self._window = None
        self._menu_item = None
        self._root_menu = "OmniCraft"

        self._world = World.instance()
        self._scene = self._world.scene
        log.info("simple_scene: Simple scene extension is loaded.")

        self._setup_scene()
        self._setup_menu()

        if self._world.timeline_callback_exists("oc.cobot.simple_scene.timeline"):
            self._world.remove_timeline_callback("oc.cobot.simple_scene.timeline")

        self._world.add_timeline_callback(
            "oc.cobot.simple_scene.timeline",
            self._on_timeline_callback,
        )

    def on_shutdown(self):
        remove_menu_items(self._menu_item, self._root_menu)
        self._cleanup()
        gc.collect()

    ##
    # Logic
    ##

    def _setup_scene(self):
        # Load asset to stage
        env_usd_path = get_proj_root_path() + "/assets/environments/default.usda"
        stage.add_reference_to_stage(env_usd_path, "/World/Env")

        cylinder_usd_path = get_proj_root_path() + "/assets/objects/cylinder.usd"
        stage.add_reference_to_stage(cylinder_usd_path, "/World/Cylinder")

        # Add prim to scene
        self._cylinder_prim = XFormPrim("/World/Cylinder", name="cylinder")
        self._cylinder_prim.set_default_state(
            position=np.array([0.8, 0, 0.08]), orientation=np.array([1, 0, 0, 0])
        )
        self._scene.add(self._cylinder_prim)
        self._scene.add(Franka(prim_path="/World/Robot", name="robot"))

        self._world.reset()

    def _setup_menu(self):
        # define menu items
        self._menu_item = [
            make_menu_item_description(
                self._ext_id,
                EXTENSION_WINDOW_NAME,
                onclick_fun=lambda a=weakref.proxy(self): a._handle_click_menu(),
            )
        ]
        add_menu_items(self._menu_item, name=self._root_menu)

    def _cleanup(self):
        # self._scene.remove_object("cylinder")
        # self._scene.remove_object("robot")
        self._scene.clear(True)
        self._world.clear_all_callbacks()

    ##
    # Handlers
    ##

    def _handle_click_menu(self):
        if self._window is None:
            return

        self._window.visible = not self._window.visible

    def _on_timeline_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            log.info("Execute callback on timeline STOP")
            self._world.reset()
