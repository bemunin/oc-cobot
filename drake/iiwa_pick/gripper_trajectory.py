from typing import List

from matplotlib import pyplot as plt
from pydrake.all import PiecewisePose


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


def plot_graph(title: str, legend: List[str], pisewise_data):
    global figure
    vec_data = pisewise_data.vector_values(pisewise_data.get_segment_times())
    plt.figure(figure)
    plt.plot(pisewise_data.get_segment_times(), vec_data.T)
    plt.title(title)
    plt.legend(legend)
    plt.show(block=False)

    figure += 1


def MakeGripperCommandTrajectory(times):
    pass


def visualize_pick_and_place_trajectory(traj_p_G, traj_R_G, traj_wsg_command, X_O):
    pass
