# from pydrake.all import()
import os

from make_gripper_frames import sketch_gripper

# import numpy as np
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

    X_O = {"initial": RigidTransform(RotationMatrix(), [0.82, -0.03, -0.046])}

    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link"), X_O["initial"])

    print(X_O["initial"])

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


if __name__ == "__main__":
    sketch_gripper(meshcat)
    input("Done exection, press Enter to exit...")
