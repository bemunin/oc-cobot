from dataclasses import asdict, dataclass
from typing import Optional

import numpy as np
import omni.graph.core as og
from oc.utils.cobot import obj_utils
from omni.isaac.core.scenes import Scene
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.franka import Franka
from omni.isaac.franka.controllers import PickPlaceController, RMPFlowController


@dataclass
class PickPlaceCmd:
    object_name: str
    place_at: np.ndarray


@dataclass
class SetGripperCmd:
    state: Optional[str] = "open"  # open or close
    position: Optional[float] = None


@dataclass
class MoveCmd:
    state: Optional[str] = None  # name of pre defined pose: [standby]
    to_point: np.ndarray = None
    rotation_deg: np.ndarray = None


@dataclass
class DelayCmd:
    time_sec: float


class FrankaManager(BaseTask):
    def __init__(self, name: str, offset: Optional[np.ndarray] = None):
        super().__init__(name, offset)
        self._franka: Franka = None
        self._pp_controller: PickPlaceController = None
        self._enable_ros = False
        self._cmds = []
        self._cmd_pointer = 0

    def set_up_scene(self, scene: Scene):
        super().set_up_scene(scene)
        self._franka = Franka(
            prim_path="/World/Franka", name="franka", position=self._offset
        )

        scene.add(self._franka)

    def post_reset(self):
        self._franka.gripper.set_joint_positions(
            self._franka.gripper.joint_closed_positions
        )
        self._pp_controller = PickPlaceController(
            name="pp_controller",
            gripper=self._franka.gripper,
            robot_articulation=self._franka,
        )
        self._rmpf_controller = RMPFlowController(
            name="rmpf_controller",
            robot_articulation=self._franka,
        )

        self._build_ros_omnigraph()

    def pre_step(self, time_step_index: int, simulation_time: float):
        if not self._enable_ros:
            self._execute(time_step_index, simulation_time)

    # Helpers
    def _execute(self, time_step_index: int, simulation_time: float):
        if self._cmd_pointer >= len(self._cmds):
            return

        cmd = self._cmds[self._cmd_pointer]

        if type(cmd) is SetGripperCmd:
            cmd = SetGripperCmd(**asdict(cmd))

            target_gripper_pos = None
            if cmd.position is not None:
                target_gripper_pos = np.clip(cmd.position, 0, 0.04)
            elif cmd.state == "close":
                target_gripper_pos = 0
            else:
                target_gripper_pos = 0.04

            action = ArticulationAction(
                joint_positions=np.full(2, target_gripper_pos),
                joint_indices=np.array([7, 8]),
            )
            self._franka.apply_action(action)

            if np.allclose(
                self._franka.gripper.get_joint_positions(),
                target_gripper_pos,
                atol=1e-3,
            ):
                self._cmd_pointer += 1
        elif type(cmd) is PickPlaceCmd:
            cmd = PickPlaceCmd(**asdict(cmd))
            target = self._scene.get_object(cmd.object_name)

            if target is None:
                return

            position, _ = target.get_world_pose()

            # adjust pick at the top of the object
            dimen = obj_utils.get_dimension(target.prim_path)
            height_m = dimen[-1]
            est_gripper_length = 0.04
            collision_offset = 0.01

            pick_at = position.copy()
            pick_pos_z = height_m - (est_gripper_length - collision_offset)
            pick_at[2] = pick_pos_z

            action = self._pp_controller.forward(
                picking_position=pick_at,
                placing_position=cmd.place_at,
                current_joint_positions=self._franka.get_joint_positions(),
            )
            self._franka.apply_action(action)

            if self._pp_controller.is_done():
                self._pp_controller.reset()
                self._cmd_pointer += 1
        elif type(cmd) is DelayCmd:
            cmd = DelayCmd(**asdict(cmd))
            if simulation_time >= cmd.time_sec:
                self._cmd_pointer += 1
        elif type(cmd) is MoveCmd:
            cmd = MoveCmd(**asdict(cmd))

            target = {}

            if cmd.state == "standby":
                target["target_end_effector_position"] = np.array([0.39, 0, 0.45])
            elif cmd.to_point is not None:
                target["target_end_effector_position"] = cmd.to_point
            else:
                target["target_end_effector_position"] = np.array([0.39, 0, 0])

            if cmd.rotation_deg is not None:
                rot_quat = euler_angles_to_quat(cmd.rotation_deg, degrees=True)
                target["target_end_effector_orientation"] = rot_quat

            action = self._rmpf_controller.forward(**target)

            self._franka.apply_action(action)

            # check if the action is done
            eef_pos, eef_orient = self._franka.end_effector.get_world_pose()

            is_pos_done = np.allclose(
                eef_pos, target["target_end_effector_position"], atol=0.06
            )
            is_orient_done = True

            if cmd.rotation_deg is not None:
                is_orient_done = np.allclose(
                    eef_orient, target["target_end_effector_orientation"], atol=0.06
                )

            if is_pos_done and is_orient_done:
                self._rmpf_controller.reset()
                self._cmd_pointer += 1

    def add_cmd(self, cmd: PickPlaceCmd | SetGripperCmd | MoveCmd | DelayCmd):
        self._cmds.append(cmd)

    @property
    def enable_ros(self):
        return self._enable_ros

    @enable_ros.setter
    def enable_ros(self, value: bool):
        self._enable_ros = value

    def _build_ros_omnigraph(self):
        keys = og.Controller.Keys
        og.Controller.edit(
            {
                "graph_path": "/ros2_graph",
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [
                    ("tick", "omni.graph.action.OnPlaybackTick"),
                    ("context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("sim_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("sub_joints", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("pub_clock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ("pub_joints", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                    (
                        "controller",
                        "omni.isaac.core_nodes.IsaacArticulationController",
                    ),
                ],
                keys.SET_VALUES: [
                    ("sub_joints.inputs:topicName", "isaac_joint_commands"),
                    ("pub_joints.inputs:targetPrim", "/World/Franka"),
                    ("pub_joints.inputs:topicName", "isaac_joint_states"),
                    ("controller.inputs:robotPath", "/World/Franka"),
                ],
                keys.CONNECT: [
                    # pub_clock
                    ("tick.outputs:tick", "pub_clock.inputs:execIn"),
                    ("context.outputs:context", "pub_clock.inputs:context"),
                    ("sim_time.outputs:simulationTime", "pub_clock.inputs:timeStamp"),
                    # pub_joints
                    ("tick.outputs:tick", "pub_joints.inputs:execIn"),
                    ("context.outputs:context", "pub_joints.inputs:context"),
                    ("sim_time.outputs:simulationTime", "pub_joints.inputs:timeStamp"),
                    # controller
                    ("tick.outputs:tick", "controller.inputs:execIn"),
                    (
                        "sub_joints.outputs:effortCommand",
                        "controller.inputs:effortCommand",
                    ),
                    ("sub_joints.outputs:jointNames", "controller.inputs:jointNames"),
                    (
                        "sub_joints.outputs:positionCommand",
                        "controller.inputs:positionCommand",
                    ),
                    (
                        "sub_joints.outputs:velocityCommand",
                        "controller.inputs:velocityCommand",
                    ),
                    # sub_joints
                    ("tick.outputs:tick", "sub_joints.inputs:execIn"),
                    ("context.outputs:context", "sub_joints.inputs:context"),
                ],
            },
        )
