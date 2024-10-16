from io import BytesIO
from typing import Optional

import cairosvg
from matplotlib import pyplot as plt
from PIL import Image
from pydrake.all import ModelVisualizer


def show_svg(plt_figure_num, svg_data):
    png_data = cairosvg.svg2png(bytestring=svg_data)
    img = Image.open(BytesIO(png_data))
    plt.figure(plt_figure_num)
    plt.imshow(img)
    plt.axis("off")
    plt.show(block=False)


def model_visualizer(meshcat, robot_model_url: Optional[str] = None):
    if not robot_model_url:
        robot_model_url = (
            "package://drake_models/franka_description/urdf/panda_arm_hand.urdf"
        )
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.AddModels(url=robot_model_url)
    visualizer.Run(loop_once=True)
    meshcat.DeleteAddedControls()


def run_sim_loop(simulator, context):
    # step the simulator in 0.1s intervals
    step = 0.1
    max_runtime_minute = 10.0
    max_runtime_sec = max_runtime_minute * 60

    while context.get_time() < max_runtime_sec:
        next_time = min(
            context.get_time() + step,
            max_runtime_sec,
        )
        simulator.AdvanceTo(next_time)
