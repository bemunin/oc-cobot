import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigs, MoveItConfigsBuilder


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # configs
    moveit_config = build_moveit_config()
    return LaunchDescription(
        [
            *args(),
            *robot_nodes(moveit_config),
            *control_nodes(moveit_config, use_sim_time=use_sim_time),
        ]
    )


def args():
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    return [declare_use_sim_time]


def robot_nodes(moveit_config: MoveItConfigs):
    # tf, tf_static
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    return [robot_state_publisher]


def control_nodes(moveit_config: MoveItConfigs, **kwargs):
    use_sim_time = kwargs.get("use_sim_time", None)

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    move_group_params = [moveit_config.to_dict(), move_group_capabilities]

    if use_sim_time:
        move_group_params.append({"use_sim_time": use_sim_time})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            move_group_capabilities,
        ],
    )
    return [ros2_control_node, *load_controllers, move_group_node]


def build_moveit_config() -> MoveItConfigs:
    config_pkg = "moveit_resources_panda_moveit_config"
    finder = FindPackageShare(package=config_pkg)
    base_path = finder.find(config_pkg)
    config_path = os.path.join(base_path, "config")

    # configs files path for panda robot
    joint_limits_file_path = os.path.join(config_path, "joint_limits.yaml")
    kinematics_file_path = os.path.join(config_path, "kinematics.yaml")
    moveit_controllers_file_path = os.path.join(
        config_path, "gripper_moveit_controllers.yaml"
    )
    srdf_model_path = os.path.join(config_path, "panda.srdf")
    pilz_cartesian_limits_file_path = os.path.join(
        config_path, "pilz_cartesian_limits.yaml"
    )

    return (
        MoveItConfigsBuilder("panda", package_name=config_pkg)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"],
            default_planning_pipeline="ompl",
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )
