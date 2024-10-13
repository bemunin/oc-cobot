# from pydrake.all import()
import os

from manipulation.station import LoadScenario, MakeHardwareStation
from pydrake.all import (
    Simulator,
    StartMeshcat,
)


meshcat = StartMeshcat()


def main():
    with open("./scenario_iiwa.dmd.yml", "r") as file:
        scenario_data = file.read()

    scenario = LoadScenario(data=scenario_data)
    station = MakeHardwareStation(
        scenario, meshcat, package_xmls=[os.getcwd() + "/package.xml"]
    )

    simulator = Simulator(station)
    context = simulator.get_mutable_context()

    x0 = station.GetOutputPort("iiwa+wsg.state_estimated").Eval(context)
    station.GetInputPort("iiwa+wsg.desired_state").FixValue(context, x0)

    # Confirm that simulation works:
    simulator.AdvanceTo(0.1)


if __name__ == "__main__":
    main()
    input("Done exection, press Enter to exit...")
