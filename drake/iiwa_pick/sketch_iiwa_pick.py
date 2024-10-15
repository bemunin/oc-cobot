import numpy as np
from gripper_trajectory import (
    MakeGripperCommandTrajectory,
    MakeGripperPoseTrajectory,
    plot_graph,
    visualize_pick_and_place_trajectory,
)
from make_gripper_frames import X_O, MakeGripperFrames, visualize_gripper_frames

# import pydot
# from helpers import show_svg
from pydrake.all import (
    RigidTransform,
    RotationMatrix,
    StartMeshcat,
)


meshcat = StartMeshcat()


def sketch_gripper():
    R_Ginitial = RotationMatrix(
        np.array(
            [
                [
                    0,
                    0.24,
                    -0.97,
                ],
                [
                    1,
                    0,
                    0,
                ],
                [
                    0,
                    -0.97,
                    -0.24,
                ],
            ]
        )
    )
    X_G = {"initial": RigidTransform(R_Ginitial, [0.4656, 0, 0.6793])}

    X_G, times = MakeGripperFrames(X_G, X_O)
    visualize_gripper_frames(X_G, X_O, meshcat)

    # create gripper trajectory from keyframes
    # traj_X_G = MakeGripperPoseTrajectory(X_G, times)

    # traj_p_G = traj_X_G.get_position_trajectory()
    # traj_R_G = traj_X_G.get_orientation_trajectory()

    # plot_graph("p_G", ["x", "y", "z"], traj_p_G)
    # plot_graph("R_G", ["qx", "qy", "qz", "qw"], traj_R_G)

    # # create end effector command trajectory
    # traj_wsg_command = MakeGripperCommandTrajectory(times)
    # plot_graph("wsg_command", None, traj_wsg_command)

    # visualize_pick_and_place_trajectory(
    #     traj_p_G, traj_R_G, traj_wsg_command, X_O, meshcat
    # )

    # traj_v_G = traj_p_G.MakeDerivative()
    # plot_graph("v_G", ["vx", "vy", "vz"], traj_v_G)


if __name__ == "__main__":
    sketch_gripper()
    input("Done exection, press Enter to exit...")
