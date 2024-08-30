from os import path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def build_pickplace_node(moveit_config: dict):
    pick_place_node = Node(
        package="cobot",
        executable="pickplace",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )
    return pick_place_node


def generate_launch_description():
    move_launch_file = path.join(
        get_package_share_directory("cobot"),
        "launch",
        "move.launch.py",
    )
    lf = open(move_launch_file).read()
    lf = lf.replace("generate_launch_description", "generate_base_launch_description")

    # Add pickplace node to to base launch description
    # This subsitution relies on the fact that
    # "*controllers," appears only once in the launch file
    # lf = lf.replace(
    #     "*controllers,", "*controllers, build_pickplace_node(moveit_config),"
    # )
    exec(lf, globals())

    return generate_base_launch_description()  # type: ignore # noqa: F821
