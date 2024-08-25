import carb  # type: ignore


def error(msg: str):
    carb.log_error(f"oc: {msg}")


def warn(msg: str):
    carb.log_warn(f"oc: {msg}")


def info(msg: str):
    carb.log_info(f"oc: {msg}")
