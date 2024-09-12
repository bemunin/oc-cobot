import omni.ui as ui
from omni.isaac.core.world.world import World
from omni.isaac.ui.element_wrappers.ui_widget_wrappers import CheckBox, StateButton
from omni.isaac.ui.style import get_style
from omni.ui import AbstractValueModel
from omni.ui import color as cl

import utils.log as log
from utils.ui import collapse_frame, vstack


def conveyor_section_ui():
    with collapse_frame("Conveyor"):
        with vstack():
            conveyor_control_btn()
            sensor_control_btn()
            ui.Spacer(height=5)
            sensor_pos_slider()
            ui.Spacer(height=5)
            sensor_visualizer()
            ui.Spacer(height=5)


def conveyor_control_btn():
    world = World.instance()
    conveyor = world.scene.get_object("conveyor")

    # Handlers
    def on_click_toggle():
        try:
            if conveyor.is_engine_on:
                conveyor.turn_off()
                log.info("Conveyor engine OFF")
            else:
                conveyor.turn_on()
                log.info("Conveyor engine ON")

        except Exception as e:
            log.error(e)
            return

    btn = StateButton(
        label="Conveyor Engine",
        a_text="ON",
        b_text="OFF",
        tooltip="Start/Stop conveyor",
        on_a_click_fn=on_click_toggle,
        on_b_click_fn=on_click_toggle,
    )

    return btn


def sensor_control_btn():
    world = World.instance()
    conveyor = world.scene.get_object("conveyor")

    # Handlers
    def on_click_toggle():
        try:
            if conveyor.is_sensor_on:
                conveyor.disable_sensor()
                log.info("Sensor disabled")
            else:
                conveyor.enable_sensor()
                log.info("Sensor enabled")

        except Exception as e:
            log.error(e)
            return

    btn = StateButton(
        label="Sensor",
        a_text="ON",
        b_text="OFF",
        tooltip="Start/Stop conveyor",
        on_a_click_fn=on_click_toggle,
        on_b_click_fn=on_click_toggle,
    )

    return btn


def sensor_pos_slider():
    conveyor = World.instance().scene.get_object("conveyor")

    # initial value
    initial_value = conveyor.get_sensor_move_distance()
    model = ui.SimpleIntModel()
    model.set_value(initial_value)

    # Handlers
    def handle_value_changed(model: AbstractValueModel):
        dist = model.get_value_as_int()
        log.info(f"Slider value changed: {dist}")

        try:
            conveyor.move_sensor(dist)
        except Exception as e:
            log.error(e)
            return

    model.add_value_changed_fn(handle_value_changed)

    # UI
    with vstack():
        ui.Label("Sensor Position", width=100, style=get_style())
        ui.Spacer()
        ui.IntSlider(
            model,
            min=-8,
            max=8,
            style={
                "background_color": cl.transparent,
                "color": cl.white,
                "draw_mode": ui.SliderDrawMode.HANDLE,
                "secondary_color": cl.grey,
                "secondary_selected_color": cl.orange,
                "font_size": 14.0,
                "border_width": 1,
                "border_color": cl.grey,
                "padding": 5,
            },
        )
        ui.Spacer(height=5)


def sensor_visualizer():
    conveyor = World.instance().scene.get_object("conveyor")
    sensor = conveyor.get_sensor_prim()

    def handle_click(value: bool):
        if value:
            sensor.show_beam_visualizer(True)
            log.info("Sensor visualizer ON")
        else:
            sensor.show_beam_visualizer(False)
            log.info("Sensor visualizer OFF")

    initial_value = sensor.is_visualize_on

    CheckBox(
        "Visualize Sensor Beam",
        default_value=initial_value,
        tooltip=" Click this checkbox to see light beam of conveyor sensor",
        on_click_fn=handle_click,
    )
