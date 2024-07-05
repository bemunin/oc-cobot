import os

import toml


def get_proj_root_path():
    """get project root path

    Returns:
        str: project root path
    """
    file_path = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(file_path, "..")


def load_config():
    root = get_proj_root_path()

    try:
        with open(f"{root}/config.local.toml", "r") as file:
            config = toml.load(file)
    except FileNotFoundError:
        with open(f"{root}/config.toml", "r") as file:
            config = toml.load(file)
    finally:
        return config
