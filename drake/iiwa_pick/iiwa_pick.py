import os
import sys

import numpy as np
import pydot
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
    Integrator,
    JacobianWrtVariable,
    LeafSystem,
    RigidTransform,
    RotationMatrix,
    Simulator,
    StartMeshcat,
    TrajectorySource,
)


meshcat = StartMeshcat()

X_O = {
    # foam brick height = 0.049, move down -0.04 = -0.089
    "initial": RigidTransform([0.6, -0.03, -0.089]),
    "goal": RigidTransform(
        RotationMatrix.MakeZRotation(np.pi / 2.0), [-0.4, 0.45, 0.049]
    ),
}


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


def createStation(builder):
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
    return station, plant


def generatePisewiseTrajectory(station, plant):
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

    traj = MakeGripperPoseTrajectory(X_G, times)
    return traj, times


def createControllerDiagram(
    builder,
    station,
    plant,
    traj,
):
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

    return integrator


def createEndeffectorCommandDiagram(builder, station, times):
    traj_wsg_command = MakeGripperCommandTrajectory(times)
    wsg_source = builder.AddSystem(TrajectorySource(traj_wsg_command))
    wsg_source.set_name("wsg.command")
    builder.Connect(wsg_source.get_output_port(), station.GetInputPort("wsg.position"))


def runSimulator(diagram, station, plant, integrator, endtime):
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

    simulator.AdvanceTo(endtime)
    meshcat.PublishRecording()

    return simulator, context


def sketch():
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
    traj_X_G = MakeGripperPoseTrajectory(X_G, times)

    traj_p_G = traj_X_G.get_position_trajectory()
    traj_R_G = traj_X_G.get_orientation_trajectory()

    plot_graph("p_G", ["x", "y", "z"], traj_p_G)
    plot_graph("R_G", ["qx", "qy", "qz", "qw"], traj_R_G)

    # # create end effector command trajectory
    traj_wsg_command = MakeGripperCommandTrajectory(times)
    plot_graph("wsg_command", None, traj_wsg_command)

    traj_v_G = traj_p_G.MakeDerivative()
    plot_graph("v_G", ["vx", "vy", "vz"], traj_v_G)


def main():
    builder = DiagramBuilder()
    station, plant = createStation(builder)

    traj, times = generatePisewiseTrajectory(station, plant)

    integrator = createControllerDiagram(builder, station, plant, traj)

    createEndeffectorCommandDiagram(builder, station, times)

    diagram = builder.Build()
    diagram.set_name("pick_and_place")
    sim_endtime = traj.get_position_trajectory().end_time()

    runSimulator(diagram, station, plant, integrator, sim_endtime)

    # Jupyter notebook, using this line to show diagram
    # RenderDiagram(diagram, max_depth=1)

    # save render diagram image
    pydot_data = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=1))[0]
    svg_data = pydot_data.create_svg()
    with open("./.diagram/diagram_output.svg", "wb") as f:
        f.write(svg_data)

    input("Done exection, press Enter to exit...")


if __name__ == "__main__":
    args = sys.argv[1:]
    mode = args[0] if args else None
    if mode == "-s":
        sketch()
    else:
        main()
    input("Done exection, press Enter to exit...")
