import os


def get_proj_root_path():
    """get project root path

    Returns:
        str: project root path
    """
    file_path = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(file_path, "..")
