import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigs, MoveItConfigsBuilder


def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # configs
    moveit_config = build_moveit_config()
    config_path, _ = get_config_path()
    initial_positions_file_path = os.path.join(config_path, "initial_positions.yaml")

    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # default to Flase
            {"use_sim_time": use_sim_time},
            {"start_state": {"content": initial_positions_file_path}},
        ],
    )

    return LaunchDescription(
        [
            *args(),
            *tool_nodes(moveit_config),
            mtc_node,
        ]
    )


def get_config_path():
    config_pkg_name = "moveit_resources_panda_moveit_config"
    finder = FindPackageShare(package=config_pkg_name)
    base_path = finder.find(config_pkg_name)
    config_path = os.path.join(base_path, "config")
    return config_path, config_pkg_name


def build_moveit_config() -> MoveItConfigs:
    config_path, config_pkg_name = get_config_path()

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
        MoveItConfigsBuilder("panda", package_name=config_pkg_name)
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


def args():
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    return [declare_use_sim_time]


def tool_nodes(moveit_config: MoveItConfigs):
    rviz_base = os.path.join(get_package_share_directory("mtc_demos"))
    rviz_full_config = os.path.join(rviz_base, "rviz", "mtc_demos.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    return [rviz_node]
