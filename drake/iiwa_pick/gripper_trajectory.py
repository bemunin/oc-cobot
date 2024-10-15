from typing import Any, List, Optional

import numpy as np
from matplotlib import pyplot as plt
from pydrake.all import PiecewisePolynomial, PiecewisePose


figure = 0


def MakeGripperPoseTrajectory(X_G, times):
    sample_times = []
    poses = []
    for name in [
        "initial",
        "prepick",
        "pick_start",
        "pick_end",
        "postpick",
        "clearance1",
        "clearance2",
        "preplace",
        "place_start",
        "place_end",
        "postplace",
    ]:
        sample_times.append(times[name])
        poses.append(X_G[name])

    return PiecewisePose.MakeLinear(sample_times, poses)


def plot_graph(title: str, legend: Optional[List[str]], pisewise_data):
    global figure
    vec_data = pisewise_data.vector_values(pisewise_data.get_segment_times())
    plt.figure(figure)
    plt.plot(pisewise_data.get_segment_times(), vec_data.T)
    plt.title(title)

    if legend:
        plt.legend(legend)
    plt.show(block=False)

    figure += 1


def MakeGripperCommandTrajectory(times):
    opened = np.array([0.107])
    closed = np.array([0.0])

    traj_wsg_command = PiecewisePolynomial.FirstOrderHold(
        [times["initial"], times["pick_start"]],
        np.hstack([[opened], [opened]]),
    )
    traj_wsg_command.AppendFirstOrderSegment(times["pick_end"], closed)
    traj_wsg_command.AppendFirstOrderSegment(times["place_start"], closed)
    traj_wsg_command.AppendFirstOrderSegment(times["place_end"], opened)
    traj_wsg_command.AppendFirstOrderSegment(times["postplace"], opened)
    return traj_wsg_command


def visualize_pick_and_place_trajectory(traj_p_G, traj_R_G, traj_wsg_command, X_O):
    pass
