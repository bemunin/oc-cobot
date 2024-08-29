import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


# Builder functions


def build_movegroup_node(moveit_config):
    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_capabilities],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return move_group_node


def build_rviz_node(moveit_config):
    rviz_config_file = os.path.join(
        get_package_share_directory("cobot"),
        "config",
        "panda_moveit_config.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return rviz_node


def build_tf_nodes(moveit_config):
    # Static TF
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    # TODO: uncomment it when use RGBD camera
    # hand2camera_tf_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=[
    #         "0.04",
    #         "0.0",
    #         "0.04",
    #         "0.0",
    #         "0.0",
    #         "0.0",
    #         "panda_hand",
    #         "sim_camera",
    #     ],
    # )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    return world2robot_tf_node, robot_state_publisher


def build_ros2_control_node():
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    return ros2_control_node


def load_controllers():
    processes = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        processes += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return processes


# Launch file declaration
def generate_launch_description():
    # args
    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    # setup
    moveit_config = (
        MoveItConfigsBuilder(
            "moveit_resources_panda",
            package_name="moveit_resources_panda_moveit_config",
        )
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration("hardware_type")
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(file_path="config/joint_limits.3x.yaml")
        .to_moveit_configs()
    )

    move_group_node = build_movegroup_node(moveit_config)
    rviz_node = build_rviz_node(moveit_config)
    world2robot_tf_node, robot_state_publisher = build_tf_nodes(moveit_config)
    ros2_control_node = build_ros2_control_node()
    controllers = load_controllers()

    return LaunchDescription(
        [
            hardware_type_arg,
            rviz_node,
            world2robot_tf_node,
            # hand2camera_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
        ]
        + controllers
    )
