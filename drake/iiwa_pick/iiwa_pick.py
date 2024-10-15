# from pydrake.all import()
import os

import numpy as np
from gripper_trajectory import (
    MakeGripperCommandTrajectory,
    MakeGripperPoseTrajectory,
    plot_graph,
)
from make_gripper_frames import MakeGripperFrames, visualize_gripper_frames

# import pydot
# from helpers import show_svg
from manipulation.station import LoadScenario, MakeHardwareStation
from pydrake.all import (
    DiagramBuilder,
    RigidTransform,
    RotationMatrix,
    Simulator,
    StartMeshcat,
)


meshcat = StartMeshcat()


def main():
    # setup scene
    with open("./scenario_iiwa.dmd.yaml", "r") as file:
        scenario_data = file.read()

    builder = DiagramBuilder()
    scenario = LoadScenario(data=scenario_data)
    station = builder.AddSystem(
        MakeHardwareStation(
            scenario, meshcat, package_xmls=[os.getcwd() + "/../package.xml"]
        )
    )
    plant = station.GetSubsystemByName("plant")

    temp_context = station.CreateDefaultContext()
    temp_plant_context = plant.GetMyContextFromRoot(temp_context)

    X_G = {
        "initial": plant.EvalBodyPoseInWorld(
            temp_plant_context, plant.GetBodyByName("body")
        )
    }
    X_O = {"initial": RigidTransform([0.82, -0.03, -0.089])}
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link"), X_O["initial"])

    print(X_O["initial"])

    print(X_G["initial"])

    # Run simulation
    diagram = builder.Build()
    diagram.set_name("iiwa_pick")

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    station_context = station.GetMyContextFromRoot(context)

    x0 = station.GetOutputPort("iiwa+wsg.state_estimated").Eval(station_context)
    station.GetInputPort("iiwa+wsg.desired_state").FixValue(station_context, x0)

    # Run simulation
    simulator.AdvanceTo(0.1)


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

    R_Ogoal = RotationMatrix.MakeZRotation(np.pi / 2.0)

    X_O = {
        # foam brick height = 0.049, move down -0.04 = -0.089
        "initial": RigidTransform([0.82, -0.03, -0.089]),
        "goal": RigidTransform(R_Ogoal, [-0.63, -0.03, 0.054]),
    }

    X_G, times = MakeGripperFrames(X_G, X_O)
    visualize_gripper_frames(X_G, X_O, meshcat)

    # create gripper trajectory from keyframes
    traj_X_G = MakeGripperPoseTrajectory(X_G, times)

    traj_p_G = traj_X_G.get_position_trajectory()
    traj_R_G = traj_X_G.get_orientation_trajectory()

    plot_graph("p_G", ["x", "y", "z"], traj_p_G)
    plot_graph("R_G", ["qx", "qy", "qz", "qw"], traj_R_G)

    # create end effector command trajectory
    traj_wsg_command = MakeGripperCommandTrajectory(times)
    plot_graph("wsg_command", None, traj_wsg_command)


if __name__ == "__main__":
    # main()
    sketch_gripper()
    input("Done exection, press Enter to exit...")
