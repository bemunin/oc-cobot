# from pydrake.all import()
import os

from manipulation.station import LoadScenario, MakeHardwareStation
from pydrake.all import Simulator, StartMeshcat


meshcat = StartMeshcat()


def MakeGripperFrames(X_WG, X_WO):
    pass


def sketch_hand():
    with open("./scene.dmd.yml", "r") as file:
        scenario_data = file.read()

    scenario = LoadScenario(data=scenario_data)
    station = MakeHardwareStation(
        scenario, meshcat, package_xmls=[os.getcwd() + "/package.xml"]
    )

    simulator = Simulator(station)

    # Confirm that simulation works:
    simulator.AdvanceTo(0.1)


def main():
    with open("./scenario_conveyor.dmd.yml", "r") as file:
        scenario_data = file.read()

    scenario = LoadScenario(data=scenario_data)
    station = MakeHardwareStation(
        scenario, meshcat, package_xmls=[os.getcwd() + "/package.xml"]
    )

    simulator = Simulator(station)
    context = simulator.get_mutable_context()

    x0 = station.GetOutputPort("panda_arm+panda_hand.state_estimated").Eval(context)
    station.GetInputPort("panda_arm+panda_hand.desired_state").FixValue(context, x0)

    # Confirm that simulation works:
    simulator.AdvanceTo(0.1)


if __name__ == "__main__":
    sketch_hand()
    input("Done exection, press Enter to exit...")
