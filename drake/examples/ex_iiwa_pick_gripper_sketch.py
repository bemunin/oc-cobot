import numpy as np
from matplotlib import pyplot as plt
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    AngleAxis,
    DiagramBuilder,
    LeafSystem,
    MeshcatVisualizer,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    Parser,
    PiecewisePolynomial,
    PiecewisePose,
    Quaternion,
    Rgba,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    StartMeshcat,
)


meshcat = StartMeshcat()


def MakeGripperFrames(X_WG, X_WO):
    """
    Takes a partial specification with X_G["initial"] and X_O["initial"] and
    X_0["goal"], and returns a X_G and times with all of the pick and place
    frames populated.
    """
    # Define (again) the gripper pose relative to the object when in grasp.
    p_GgraspO = [0, 0.12, 0]
    R_GgraspO = RotationMatrix.MakeXRotation(
        np.pi / 2.0
    ) @ RotationMatrix.MakeZRotation(np.pi / 2.0)
    X_GgraspO = RigidTransform(R_GgraspO, p_GgraspO)
    X_OGgrasp = X_GgraspO.inverse()
    # pregrasp is negative y in the gripper frame (see the figure!).
    X_GgraspGpregrasp = RigidTransform([0, -0.08, 0])

    X_WG["pick"] = X_WO["initial"] @ X_OGgrasp
    X_WG["prepick"] = X_WG["pick"] @ X_GgraspGpregrasp
    X_WG["place"] = X_WO["goal"] @ X_OGgrasp
    X_WG["preplace"] = X_WG["place"] @ X_GgraspGpregrasp

    # I'll interpolate a halfway orientation by converting to axis angle and halving the angle.
    X_GprepickGpreplace = X_WG["prepick"].inverse() @ X_WG["preplace"]
    angle_axis = X_GprepickGpreplace.rotation().ToAngleAxis()
    X_GprepickGclearance = RigidTransform(
        AngleAxis(angle=angle_axis.angle() / 2.0, axis=angle_axis.axis()),
        X_GprepickGpreplace.translation() / 2.0 + np.array([0, -0.3, 0]),
    )
    X_WG["clearance"] = X_WG["prepick"] @ X_GprepickGclearance

    # Now let's set the timing
    times = {"initial": 0}
    X_GinitialGprepick = X_G["initial"].inverse() @ X_WG["prepick"]
    times["prepick"] = times["initial"] + 10.0 * np.linalg.norm(
        X_GinitialGprepick.translation()
    )
    # Allow some time for the gripper to close.
    times["pick_start"] = times["prepick"] + 2.0
    times["pick_end"] = times["pick_start"] + 2.0
    X_WG["pick_start"] = X_WG["pick"]
    X_WG["pick_end"] = X_WG["pick"]
    times["postpick"] = times["pick_end"] + 2.0
    X_WG["postpick"] = X_WG["prepick"]
    time_to_from_clearance = 10.0 * np.linalg.norm(X_GprepickGclearance.translation())
    times["clearance"] = times["postpick"] + time_to_from_clearance
    times["preplace"] = times["clearance"] + time_to_from_clearance
    times["place_start"] = times["preplace"] + 2.0
    times["place_end"] = times["place_start"] + 2.0
    X_WG["place_start"] = X_WG["place"]
    X_WG["place_end"] = X_WG["place"]
    times["postplace"] = times["place_end"] + 2.0
    X_WG["postplace"] = X_WG["preplace"]

    return X_WG, times


def visualize_gripper_frames(X_G, X_O):
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    for key, pose in X_G.items():
        g = parser.AddModelsFromUrl(
            "package://drake_models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
        )[0]
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("body", g), pose)
    for key, pose in X_O.items():
        o = parser.AddModelsFromUrl(
            "package://drake_models/manipulation_station/061_foam_brick.sdf"
        )[0]
        plant.WeldFrames(
            plant.world_frame(), plant.GetFrameByName("base_link", o), pose
        )

    plant.Finalize()

    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.ForcedPublish(context)


def MakeGripperPoseTrajectory(X_G, times):
    """
    Constructs a gripper position trajectory from the plan "sketch".
    """

    sample_times = []
    poses = []
    for name in [
        "initial",
        "prepick",
        "pick_start",
        "pick_end",
        "postpick",
        "clearance",
        "preplace",
        "place_start",
        "place_end",
        "postplace",
    ]:
        sample_times.append(times[name])
        poses.append(X_G[name])

    return PiecewisePose.MakeLinear(sample_times, poses)


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


class GripperTrajectoriesToPosition(LeafSystem):
    def __init__(self, plant, traj_p_G, traj_R_G, traj_wsg_command):
        LeafSystem.__init__(self)
        self.plant = plant
        self.gripper_body = plant.GetBodyByName("body")
        self.left_finger_joint = plant.GetJointByName("left_finger_sliding_joint")
        self.right_finger_joint = plant.GetJointByName("right_finger_sliding_joint")
        self.traj_p_G = traj_p_G
        self.traj_R_G = traj_R_G
        self.traj_wsg_command = traj_wsg_command
        self.plant_context = plant.CreateDefaultContext()

        self.DeclareVectorOutputPort(
            "position", plant.num_positions(), self.CalcPositionOutput
        )

    def CalcPositionOutput(self, context, output):
        t = context.get_time()
        X_G = RigidTransform(Quaternion(self.traj_R_G.value(t)), self.traj_p_G.value(t))
        self.plant.SetFreeBodyPose(self.plant_context, self.gripper_body, X_G)
        wsg = self.traj_wsg_command.value(t)[0, 0]
        self.left_finger_joint.set_translation(self.plant_context, -wsg / 2.0)
        self.right_finger_joint.set_translation(self.plant_context, wsg / 2.0)
        output.SetFromVector(self.plant.GetPositions(self.plant_context))


def visualize_pick_and_place_trajectory(traj_p_G, traj_R_G, traj_wsg_command, X_O):
    builder = DiagramBuilder()

    # Note: Don't use AddMultibodyPlantSceneGraph because we are only using
    # MultibodyPlant for parsing, then wiring directly to SceneGraph.
    scene_graph = builder.AddSystem(SceneGraph())
    plant = MultibodyPlant(time_step=0.0)
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    gripper = parser.AddModelsFromUrl(
        "package://drake_models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
    )[0]
    brick = "package://drake_models/manipulation_station/061_foam_brick.sdf"
    for key, pose in X_O.items():
        o = parser.AddModelsFromUrl(brick)[0]
        plant.WeldFrames(
            plant.world_frame(), plant.GetFrameByName("base_link", o), pose
        )
    plant.Finalize()

    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(
        to_pose.get_output_port(),
        scene_graph.get_source_pose_port(plant.get_source_id()),
    )

    traj_to_position = builder.AddSystem(
        GripperTrajectoriesToPosition(plant, traj_p_G, traj_R_G, traj_wsg_command)
    )
    builder.Connect(traj_to_position.get_output_port(), to_pose.get_input_port())

    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    diagram = builder.Build()

    simulator = Simulator(diagram)
    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(traj_p_G.end_time())
    meshcat.PublishRecording()


X_G = {
    "initial": RigidTransform(
        RotationMatrix.MakeXRotation(-np.pi / 2.0), [0, -0.25, 0.25]
    )
}

X_O = {
    "initial": RigidTransform(
        RotationMatrix.MakeZRotation(np.pi / 2.0), [-0.2, -0.75, 0.025]
    ),
    "goal": RigidTransform(RotationMatrix.MakeZRotation(np.pi), [0.75, 0, 0.025]),
}
X_G, times = MakeGripperFrames(X_G, X_O)
print(
    f"Sanity check: The entire maneuver will take {times['postplace']} seconds to execute."
)

visualize_gripper_frames(X_G, X_O)
traj_X_G = MakeGripperPoseTrajectory(X_G, times)

# Plot trajectory

traj_p_G = traj_X_G.get_position_trajectory()

p_G = traj_p_G.vector_values(traj_p_G.get_segment_times())
plt.figure(0)
plt.plot(traj_p_G.get_segment_times(), p_G.T)
plt.legend(["x", "y", "z"])
plt.title("p_G")

plt.show(block=False)

meshcat.ResetRenderMode()
meshcat.SetLine("p_G", p_G, 2.0, rgba=Rgba(1, 0.65, 0))

traj_R_G = traj_X_G.get_orientation_trajectory()
R_G = traj_R_G.vector_values(traj_R_G.get_segment_times())
plt.figure(1)
plt.plot(traj_R_G.get_segment_times(), R_G.T)
plt.legend(["qx", "qy", "qz", "qw"])
plt.title("R_G")
plt.show(block=False)

traj_wsg_command = MakeGripperCommandTrajectory(times)

wsg_command = traj_wsg_command.vector_values(traj_wsg_command.get_segment_times())
plt.figure(2)
plt.plot(traj_wsg_command.get_segment_times(), wsg_command.T)
plt.title("wsg_command")
plt.show(block=False)

visualize_pick_and_place_trajectory(traj_p_G, traj_R_G, traj_wsg_command, X_O)

traj_v_G = traj_p_G.MakeDerivative()

v_G = traj_v_G.vector_values(traj_v_G.get_segment_times())
plt.figure(3)
plt.plot(traj_v_G.get_segment_times(), v_G.T)
plt.legend(["vx", "vy", "vz"])
plt.title("v_G")
plt.show(block=False)


input("Done exection, press Enter to exit...")
