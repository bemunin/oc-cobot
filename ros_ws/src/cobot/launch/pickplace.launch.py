from os import path

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    move_launch_file = path.join(
        get_package_share_directory("cobot"),
        "launch",
        "move.launch.py",
    )
    lf = open(move_launch_file).read()
    lf = lf.replace("generate_launch_description", "generate_base_launch_description")
    exec(lf, globals())

    return generate_base_launch_description()  # type: ignore # noqa: F821
