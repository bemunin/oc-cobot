from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigs

from launch.actions import DeclareLaunchArgument


def build_pickplace_node(moveit_config: MoveItConfigs) -> List:
    pick_place_node = Node(
        package="cobot",
        executable="pickplace",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )
    return [pick_place_node]


def modify_create_is_use_mtc_arg():
    return DeclareLaunchArgument(
        "is_use_mtc",
        default_value="true",
        description="boolean flag to specify whether to use MoveItTaskConstructor settings",
    )


def generate_launch_description():
    move_launch_file = path.join(
        get_package_share_directory("cobot"),
        "launch",
        "move.launch.py",
    )
    lf = open(move_launch_file).read()

    lf = lf.replace("generate_launch_description", "generate_base_launch_description")

    # The subsitution in this file relies on the fact that
    # the replace string appears only once in the launch file

    # Modify is_use_mtc_arg default argument
    lf = lf.replace(
        "is_use_mtc_arg = create_is_use_mtc_arg()",
        "is_use_mtc_arg = modify_create_is_use_mtc_arg()",
    )

    lf = lf.replace('default_value="isaac"', 'default_value="mock_components"')

    lf = lf.replace(
        "entities = setup_entities(moveit_config)",
        "entities = build_pickplace_node(moveit_config)",
    )

    exec(lf, globals())

    return generate_base_launch_description()  # type: ignore # noqa: F821
