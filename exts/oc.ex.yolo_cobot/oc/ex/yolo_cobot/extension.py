import gc
import weakref

import omni
import omni.ui as ui
from omni.isaac.core import World
from omni.isaac.ui.menu import make_menu_item_description
from omni.kit.menu.utils.scripts.utils import add_menu_items, remove_menu_items

import utils.log as log
from utils.ui import vstack

from .conveyor import Conveyor, conveyor_section_ui
from .spawner import SpawnerTask


EXTENSION_WINDOW_NAME = "YoloCobot Control Panel"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        self._window = None
        self._root_menu = "OmniCraft"
        self._panel_menu_item = None
        self._spawner_task = None

        # initialize
        self._setup_scene()
        self._create_menus()
        self._build_ui()

    def on_shutdown(self):
        remove_menu_items(self._setting_menu_item, self._root_menu)
        self._cleanup_scene()
        gc.collect()

    def _create_menus(self):
        # define menu items
        self._setting_menu_item = [
            make_menu_item_description(
                self._ext_id,
                EXTENSION_WINDOW_NAME,
                lambda a=weakref.proxy(self): a._handle_click_control_panel_menu_item(),
            )
        ]

        # add menu items to root menu
        add_menu_items(self._setting_menu_item, name=self._root_menu)

    def _build_ui(self):
        try:
            self._window = ui.Window(
                EXTENSION_WINDOW_NAME,
                width=350,  # px
                height=0,  # auto
                visible=True,
                dockPreference=ui.DockPreference.LEFT_BOTTOM,
            )

            with self._window.frame:
                with vstack():
                    conveyor_section_ui()

            return
        except Exception as e:
            log.error(f"(Extension) Failed to build UI; Error message = {e}")

    ##
    # Scene
    ##
    def _setup_scene(self):
        world = World.instance()
        scene = world.scene

        # setup conveyor
        conveyor = Conveyor("conveyor", "/Main/Conveyor")
        conveyor.turn_on()
        scene.add(conveyor)

        # setup spawner task
        spawner_task = SpawnerTask()
        world.add_task(spawner_task)

        # reset world to build task set_up_scene
        # must call to make task work
        world.reset()

    def _cleanup_scene(self):
        world = World.instance()
        scene = world.scene

        # remove objects
        scene.remove_object("conveyor")

        # remove tasks

        current_tasks = world.get_current_tasks()
        del current_tasks[SpawnerTask.name]

        # clear callbacks
        world.clear_all_callbacks()

    ##
    # Handlers
    ##
    def _handle_click_control_panel_menu_item(self):
        if self._window is None:
            return

        self._window.visible = not self._window.visible
