import numpy as np
from manipulation.station import LoadScenario, MakeHardwareStation
from pydrake.all import (
    DiagramBuilder,
    Integrator,
    JacobianWrtVariable,
    LeafSystem,
    Simulator,
    StartMeshcat,
)


scenario_data = """
directives:
- add_directives:
    file: package://manipulation/iiwa_and_wsg.dmd.yaml
model_drivers:
    iiwa: !IiwaDriver
      control_mode: position_and_torque
      hand_model_name: wsg
    wsg: !SchunkWsgDriver {}
"""


class PseudoInverseController(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._iiwa = plant.GetModelInstanceByName("iiwa")
        self._G = plant.GetBodyByName("body").body_frame()
        self._W = plant.world_frame()

        self.DeclareVectorInputPort("iiwa.position", 7)
        self.DeclareVectorOutputPort("iiwa.velocity", 7, self.CalcOutput)

    def CalcOutput(self, context, output):
        q = self.get_input_port().Eval(context)
        self._plant.SetPositions(self._plant_context, self._iiwa, q)
        J_G = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context,
            JacobianWrtVariable.kQDot,
            self._G,
            [0, 0, 0],
            self._W,
            self._W,
        )
        J_G = J_G[:, 0:7]  # Ignore gripper terms

        V_G_desired = np.array(
            [
                0,  # rotation about x
                -0.1,  # rotation about y
                0,  # rotation about z
                0,  # x
                -0.05,  # y
                -0.1,
            ]
        )  # z
        v = np.linalg.pinv(J_G).dot(V_G_desired)
        output.SetFromVector(v)


def jacobian_controller_example():
    meshcat = StartMeshcat()
    builder = DiagramBuilder()

    scenario = LoadScenario(data=scenario_data)
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat=meshcat))
    plant = station.GetSubsystemByName("plant")

    controller = builder.AddSystem(PseudoInverseController(plant))
    integrator = builder.AddSystem(Integrator(7))

    builder.Connect(controller.get_output_port(), integrator.get_input_port())
    builder.Connect(integrator.get_output_port(), station.GetInputPort("iiwa.position"))
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        controller.get_input_port(),
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    station_context = station.GetMyContextFromRoot(context)
    station.GetInputPort("iiwa.torque").FixValue(station_context, np.zeros((7, 1)))
    station.GetInputPort("wsg.position").FixValue(station_context, [0.1])
    integrator.set_integral_value(
        integrator.GetMyContextFromRoot(context),
        plant.GetPositions(
            plant.GetMyContextFromRoot(context),
            plant.GetModelInstanceByName("iiwa"),
        ),
    )

    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(5.0)
    meshcat.PublishRecording()


def main():
    jacobian_controller_example()


if __name__ == "__main__":
    main()
    input("Done execution, press Enter to exit...")
