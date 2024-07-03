import omni.ui as ui
from omni.isaac.ui.style import get_style
from omni.ui import CollapsableFrame


def collapse_frame(title: str, default_close=False):
    return CollapsableFrame(
        title,
        width=ui.Fraction(1),
        height=0,
        collapsed=default_close,
        style=get_style(),
    )


def vstack():
    return ui.VStack(style=get_style(), spacing=5, height=0)
