import omni.ui as ui
from omni.isaac.core.world.world import World
from omni.isaac.ui.element_wrappers.ui_widget_wrappers import CheckBox
from omni.isaac.ui.ui_utils import add_line_rect_flourish, btn_builder, format_tt
from omni.kit.window.property.templates import LABEL_WIDTH
from omni.ui import AbstractItemModel

import utils.log as log
from utils.ui import collapse_frame, vstack

from .enums import ObjectItem, ObjectType
from .task import SpawnerTask


def spawner_task_section_ui():
    with collapse_frame("Spawner Task", default_close=False):
        with vstack():
            type_model = object_type_dropdown()
            object_spawn_dropdown(type_model)
            ui.Spacer(height=10)
            random_angle_checkbox()
            ui.Spacer(height=2)
            random_position_checkbox()
            ui.Spacer(height=10)
            reset_button()
            ui.Spacer(height=5)


##
# Components
##


def reset_button():
    def handle_click():
        task = World.instance().get_task(SpawnerTask.name)
        task.cleanup()

    btn_builder(label="Reset Execution", text="RESET", on_clicked_fn=handle_click)


def object_type_dropdown() -> AbstractItemModel:
    task = World.instance().get_task(SpawnerTask.name)

    def handle_select(value_index: int):
        enum_list = list(ObjectType)
        try:
            selected_item = enum_list[value_index]
        except KeyError:
            log.log_warning("(spawner-ui) Error value index excceed ObjectType size")
        else:
            task.object_type = selected_item

    dropdown_items = to_dropdown_string_items(ObjectType)

    # get the initial value from the task
    initial_object_type = task.object_type.name.replace("_", " ").capitalize()
    default_val = 0
    try:
        default_val = dropdown_items.index(initial_object_type)
    except ValueError:
        pass

    model = custom_dropdown_builder(
        label="Object Type",
        default_val=default_val,
        items=dropdown_items,
        on_clicked_fn=handle_select,
    )

    return model


def object_spawn_dropdown(type_model: AbstractItemModel):
    """UI to select object spawn item from dropdown.
    data will be one item (ObjectItem.REAL_BANANA) or all (random_object)
    """

    # use to lock handle_select when updating dropdown items caused by
    # user changing object type dropdown
    is_updating_content = False
    task = World.instance().get_task(SpawnerTask.name)

    filter_prefix = get_object_type_prefix(type_model)
    dropdown_items = to_dropdown_string_items(ObjectItem, filter_prefix)

    # object items handler
    def handle_select(value_index: int):
        nonlocal is_updating_content

        if is_updating_content:
            return

        type_prefix = get_object_type_prefix(type_model)
        item_list = [
            i
            for i in list(ObjectItem)
            if i.name.lower().startswith(type_prefix) or i.name.lower() == "all"
        ]

        object_item = ObjectItem.ALL

        try:
            object_item = item_list[value_index]
        except Exception as e:
            log.error(f"(spawner-ui) Error {e}")
        finally:
            task.object_spawn = object_item

    default_val = 0
    initial_spawn_obj = get_dropdown_item_name(task.object_spawn, filter_prefix)

    try:
        default_val = dropdown_items.index(initial_spawn_obj)
    except ValueError:
        pass

    obj_item_model = custom_dropdown_builder(
        label="Object Spawn",
        default_val=default_val,
        items=dropdown_items,
        on_clicked_fn=handle_select,
    )

    def handle_object_type_changed(model, item):
        nonlocal is_updating_content
        is_updating_content = True

        # update object item dropdown item list
        for i in obj_item_model.get_item_children():
            obj_item_model.remove_item(i)

        filter_prefix = get_object_type_prefix(type_model)

        items = to_dropdown_string_items(ObjectItem, filter_prefix)

        for i in items:
            obj_item_model.append_child_item(None, ui.SimpleStringModel(i))

        is_updating_content = False

        # reset the value to ObjectItem.ALL
        all_value_choice = 0
        obj_item_model.get_item_value_model().set_value(all_value_choice)

    type_model.add_item_changed_fn(handle_object_type_changed)


def random_position_checkbox():
    task = World.instance().get_task(SpawnerTask.name)

    def handle_click(value: bool):
        task.is_random_pos = value

    initial_value = task.is_random_pos

    CheckBox(
        "Random Position",
        default_value=initial_value,
        tooltip=" Click this checkbox to random objects' position in the belt along y-axis",
        on_click_fn=handle_click,
    )


def random_angle_checkbox():
    task = World.instance().get_task(SpawnerTask.name)

    def handle_click(value: bool):
        task.is_random_angle = value

    initial_value = task.is_random_angle

    CheckBox(
        "Random Angle",
        default_value=initial_value,
        tooltip=" Click this checkbox to random objects' angle in the belt along z-axis",
        on_click_fn=handle_click,
    )


##
# Utils
##


def to_dropdown_string_items(
    enum_class: ObjectType | ObjectItem, filter_prefix: str = None
) -> list[str]:
    """format item names in enum to dropdown string display in UI

    Args:
        enum_class (ObjectType | ObjectItem): _description_

    Returns:
        _type_: _description_
    """

    def filter(i: str):
        return i.lower().startswith(filter_prefix) or i.lower() == "all"

    if filter_prefix is None:
        return [i.name.replace("_", " ").capitalize() for i in enum_class]
    else:
        return [
            get_dropdown_item_name(i, filter_prefix)
            for i in enum_class
            if filter(i.name)
        ]


def get_dropdown_item_name(
    item: ObjectType | ObjectItem, filter_prefix: str = None
) -> str:
    return item.name.lower().replace(filter_prefix, "").replace("_", " ").capitalize()


def get_object_type_prefix(object_type_model: AbstractItemModel):
    value = object_type_model.get_item_value_model().as_int

    if value == ObjectType.GEOMETRIC.value:
        return "geo_"
    else:
        return "real_"


def custom_dropdown_builder(
    label="",
    type="dropdown",
    default_val=0,
    items=["Option 1", "Option 2", "Option 3"],
    tooltip="",
    on_clicked_fn=None,
):
    """
    Creates a Stylized Dropdown Combobox.
    This is a custom version of the dropdown_builder from omni.isaac.ui.ui_utils
    by modifying on_clicked_wrapper to pass the index of the selected item.
    instead of the selected item name


    Args:
        label (str, optional): Label to the left of the UI element. Defaults to "".
        type (str, optional): Type of UI element. Defaults to "dropdown".
        default_val (int, optional): Default index of dropdown items. Defaults to 0.
        items (list, optional): List of items for dropdown box. Defaults to ["Option 1", "Option 2", "Option 3"].
        tooltip (str, optional): Tooltip to display over the Label. Defaults to "".
        on_clicked_fn (Callable, optional): Call-back function when clicked. Defaults to None.

    Returns:
        AbstractItemModel: model
    """
    with ui.HStack():
        ui.Label(
            label,
            width=LABEL_WIDTH,
            alignment=ui.Alignment.LEFT_CENTER,
            tooltip=format_tt(tooltip),
        )
        combo_box = ui.ComboBox(
            default_val,
            *items,
            name="ComboBox",
            width=ui.Fraction(1),
            alignment=ui.Alignment.LEFT_CENTER,
        ).model
        add_line_rect_flourish(False)

        def on_clicked_wrapper(model, val):
            on_clicked_fn(model.get_item_value_model().as_int)

        if on_clicked_fn is not None:
            combo_box.add_item_changed_fn(on_clicked_wrapper)

    return combo_box
