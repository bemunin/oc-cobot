import os

import numpy as np
from gripper_trajectory import (
    MakeGripperCommandTrajectory,
    MakeGripperPoseTrajectory,
)
from make_gripper_frames import X_O, MakeGripperFrames

# import pydot
# from helpers import show_svg
from manipulation.station import LoadScenario, MakeHardwareStation
from pydrake.all import (
    DiagramBuilder,
    Integrator,
    JacobianWrtVariable,
    LeafSystem,
    Simulator,
    StartMeshcat,
    TrajectorySource,
)


meshcat = StartMeshcat()


class PseudoInverseController(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._iiwa = plant.GetModelInstanceByName("iiwa")
        self._G = plant.GetBodyByName("body").body_frame()
        self._W = plant.world_frame()

        self.V_G_port = self.DeclareVectorInputPort("V_WG", 6)
        self.q_port = self.DeclareVectorInputPort("iiwa.position", 7)
        self.DeclareVectorOutputPort("iiwa.velocity", 7, self.CalcOutput)
        self.iiwa_start = plant.GetJointByName("iiwa_joint_1").velocity_start()
        self.iiwa_end = plant.GetJointByName("iiwa_joint_7").velocity_start()

    def CalcOutput(self, context, output):
        V_G = self.V_G_port.Eval(context)
        q = self.q_port.Eval(context)
        self._plant.SetPositions(self._plant_context, self._iiwa, q)
        J_G = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context,
            JacobianWrtVariable.kV,
            self._G,
            [0, 0, 0],
            self._W,
            self._W,
        )
        J_G = J_G[:, self.iiwa_start : self.iiwa_end + 1]  # Only iiwa terms.
        v = np.linalg.pinv(J_G).dot(V_G)
        output.SetFromVector(v)


def main():
    builder = DiagramBuilder()

    # setup scene
    with open("./scenario_iiwa.dmd.yaml", "r") as file:
        scenario_data = file.read()
    scenario = LoadScenario(data=scenario_data)
    station = builder.AddSystem(
        MakeHardwareStation(
            scenario, meshcat, package_xmls=[os.getcwd() + "/../package.xml"]
        )
    )

    plant = station.GetSubsystemByName("plant")
    plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base_link"), X_O["initial"])

    # state 1: make gripper frame
    temp_context = station.CreateDefaultContext()
    temp_plant_context = plant.GetMyContextFromRoot(temp_context)
    X_G = {
        "initial": plant.EvalBodyPoseInWorld(
            temp_plant_context, plant.GetBodyByName("body")
        )
    }
    X_O["initial"] = plant.EvalBodyPoseInWorld(
        temp_plant_context, plant.GetBodyByName("base_link")
    )
    X_G, times = MakeGripperFrames(X_G, X_O)

    # state 2: add the gripper's trajectory
    traj = MakeGripperPoseTrajectory(X_G, times)
    traj_p_G = traj.get_position_trajectory()
    traj_V_G = traj.MakeDerivative()

    V_G_source = builder.AddSystem(TrajectorySource(traj_V_G))
    V_G_source.set_name("v_WG")
    controller = builder.AddSystem(PseudoInverseController(plant))
    controller.set_name("PseudoInverseController")
    builder.Connect(V_G_source.get_output_port(), controller.GetInputPort("V_WG"))

    integrator = builder.AddSystem(Integrator(7))
    integrator.set_name("integrator")
    builder.Connect(controller.get_output_port(), integrator.get_input_port())
    builder.Connect(integrator.get_output_port(), station.GetInputPort("iiwa.position"))
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        controller.GetInputPort("iiwa.position"),
    )

    # state 3: add the end-effector's trajectory
    traj_wsg_command = MakeGripperCommandTrajectory(times)
    wsg_source = builder.AddSystem(TrajectorySource(traj_wsg_command))
    wsg_source.set_name("wsg.command")
    builder.Connect(wsg_source.get_output_port(), station.GetInputPort("wsg.position"))

    # state 4: run simulation
    diagram = builder.Build()
    diagram.set_name("pick_and_place")

    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    _ = station.GetMyContextFromRoot(context)
    integrator.set_integral_value(
        integrator.GetMyContextFromRoot(context),
        plant.GetPositions(
            plant.GetMyContextFromRoot(context),
            plant.GetModelInstanceByName("iiwa"),
        ),
    )

    diagram.ForcedPublish(context)
    meshcat.StartRecording()
    simulator.AdvanceTo(traj_p_G.end_time())
    meshcat.PublishRecording()

    # RenderDiagram(diagram, max_depth=1)
    input("Done exection, press Enter to exit...")


if __name__ == "__main__":
    main()
    input("Done exection, press Enter to exit...")
