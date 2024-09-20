import gc
import weakref
from collections import namedtuple

import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import stage
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.franka.controllers.rmpflow_controller import RMPFlowController
from omni.isaac.ui.menu import make_menu_item_description
from omni.kit.menu.utils.scripts.utils import add_menu_items, remove_menu_items

from utils import get_proj_root_path, log


EXTENSION_WINDOW_NAME = "Simple Scene Panel"


Pose = namedtuple("Pose", ["position", "orientation"])


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id

        # robot stage
        self._stage = "move_cylinder"
        self._robot = None
        self._target_item = None
        self._cylinder = None
        self._cube = None
        self._standby_joint_positions = None

        # stage
        self._count = 0

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

        if self._world.physics_callback_exists("oc.cobot.simple_scene.physics"):
            self._world.remove_physics_callback("oc.cobot.simple_scene.physics")

        self._world.add_physics_callback(
            "oc.cobot.simple_scene.physics",
            self._on_physics_callback,
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

        cylinder_usd_path = get_proj_root_path() + "/assets/objects/cube.usd"
        stage.add_reference_to_stage(cylinder_usd_path, "/World/Cube")

        # Add prim to scene
        cylinder_pose = Pose(np.array([0.5, 0, 0.08]), np.array([1, 0, 0, 0]))
        self._cylinder = XFormPrim(
            "/World/Cylinder",
            name="cylinder",
        )
        self._cylinder.set_default_state(
            position=cylinder_pose.position, orientation=cylinder_pose.orientation
        )

        cube_pose = Pose(np.array([0.5, 0.5, 0.02]), np.array([1, 0, 0, 0]))
        self._cube = XFormPrim(
            "/World/Cube",
            name="cube",
        )
        self._cube.set_default_state(
            position=cube_pose.position, orientation=cube_pose.orientation
        )
        # Add objects to scene
        self._scene.clear(True)

        self._scene.add(self._cylinder)
        self._scene.add(self._cube)

        self._robot = Franka(prim_path="/World/Robot", name="robot")
        self._scene.add(self._robot)

        self._world.reset()

        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )

        # set up robot controller
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self._robot.gripper,
            robot_articulation=self._robot,
        )

        self._standby_joint_positions = self._robot.get_joint_positions().copy()

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
    # Handlers & Callbacks
    ##

    def _handle_click_menu(self):
        if self._window is None:
            return

        self._window.visible = not self._window.visible

    def _on_timeline_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            log.info("Execute callback on timeline STOP")
            # self._world.reset()

    def _on_physics_callback(self, step_size):
        if self._stage == "move_cylinder":
            item_position, _ = self._cylinder.get_world_pose()
            item_position[2] = 0.16 - 0.02
            goal_position = np.array([0, 0.8, 0.16 - 0.02])
            current_joint_positions = self._robot.get_joint_positions()

            actions = self._controller.forward(
                picking_position=item_position,
                placing_position=goal_position,
                current_joint_positions=current_joint_positions,
            )
            self._robot.apply_action(actions)

            # Only for the pick and place controller, indicating if the state
            # machine reached the final state.
            if self._controller.is_done():
                self._controller.reset()
                self._stage = "move_cube"
        elif self._stage == "move_cube":
            item_position, _ = self._cube.get_world_pose()
            goal_position = np.array([0, 0.5, 0.02])
            current_joint_positions = self._robot.get_joint_positions()

            actions = self._controller.forward(
                picking_position=item_position,
                placing_position=goal_position,
                current_joint_positions=current_joint_positions,
            )
            self._robot.apply_action(actions)
            # Only for the pick and place controller, indicating if the state
            # machine reached the final state.
            if self._controller.is_done():
                self._stage = "to_standby"
        elif self._stage == "to_standby":
            self._controller = RMPFlowController(
                name="robot_rmpflow_controller", robot_articulation=self._robot
            )

            target_position = np.array([0.39, 0, 0.45])
            actions = self._controller.forward(
                target_end_effector_position=target_position
            )

            self._robot.apply_action(actions)

            current_eef_position, _ = self._robot.end_effector.get_world_pose()

            diff = np.linalg.norm(target_position - current_eef_position)
            log.info(f"simple_scene: diff: {diff}")
            if diff < 0.06:
                self._stage = "done"
                log.info("simple_scene: Done executing move items.")
                self._world.pause()
