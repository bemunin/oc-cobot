import os

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    AngleAxis,
    DiagramBuilder,
    MeshcatVisualizer,
    Parser,
    RigidTransform,
    RotationMatrix,
)


def visualize_gripper_frames(X_G, X_O, meshcat):
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    parser = Parser(plant, scene_graph)
    parser.package_map().AddPackageXml(f"{os.getcwd()}/../package.xml")
    parser.SetAutoRenaming(True)

    parser.AddModelsFromUrl("package://oc_cobot/scene.dmd.yaml")

    for _, pose in X_G.items():
        g = parser.AddModelsFromUrl(
            "package://drake_models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
        )[0]
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("body", g), pose)

    for _, pose in X_O.items():
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


def MakeGripperFrames(X_WG, X_WO):
    times = {}

    p_GpickO = [0, 0.12, 0]
    R_GpickO = RotationMatrix.MakeXRotation(np.pi / 2.0) @ RotationMatrix.MakeZRotation(
        np.pi / 2.0
    )
    X_GpickO = RigidTransform(R_GpickO, p_GpickO)
    X_OGpick = X_GpickO.inverse()

    # frames
    X_WG["pick"] = X_WO["initial"] @ X_OGpick

    X_GpickGprepick = RigidTransform([0, -0.08, 0])
    X_WG["prepick"] = X_WG["pick"] @ X_GpickGprepick
    X_WG["place"] = X_WO["goal"] @ X_OGpick
    X_WG["preplace"] = X_WG["place"] @ X_GpickGprepick

    X_GprepickGpreplace = X_WG["prepick"].inverse() @ X_WG["preplace"]
    angle_axis = X_GprepickGpreplace.rotation().ToAngleAxis()
    X_GprepickGclearance1 = RigidTransform(
        AngleAxis(angle=angle_axis.angle() / 2.0, axis=angle_axis.axis()),
        X_GprepickGpreplace.translation() / 2.0 + np.array([-0.5, -0.35, 0]),
    )
    X_WG["clearance1"] = X_WG["prepick"] @ X_GprepickGclearance1

    X_Gclearance1Gclearance2 = RigidTransform(
        AngleAxis(angle=angle_axis.angle() / 2.0, axis=angle_axis.axis()),
        X_GprepickGpreplace.translation() / 2.0 + np.array([-0.12, 0, -0.12]),
    )

    X_WG["clearance2"] = X_WG["clearance1"] @ X_Gclearance1Gclearance2

    # times
    times = {"initial": 0}
    X_GinitialGprepick = X_WG["initial"].inverse() @ X_WG["prepick"]
    times["prepick"] = times["initial"] + 5.0 * np.linalg.norm(
        X_GinitialGprepick.translation()
    )

    # Allow some time for the gripper to close.
    times["pick_start"] = times["prepick"] + 2.0
    times["pick_end"] = times["pick_start"] + 2.0
    X_WG["pick_start"] = X_WG["pick"]
    X_WG["pick_end"] = X_WG["pick"]
    times["postpick"] = times["pick_end"] + 2.0
    X_WG["postpick"] = X_WG["prepick"]

    time_to_from_clearance1 = 5.0 * np.linalg.norm(X_GprepickGclearance1.translation())
    times["clearance1"] = times["postpick"] + time_to_from_clearance1

    time_to_from_clearance2 = 5.0 * np.linalg.norm(
        X_Gclearance1Gclearance2.translation()
    )

    times["clearance2"] = times["clearance1"] + time_to_from_clearance2
    times["preplace"] = times["clearance2"] + 2.0
    times["place_start"] = times["preplace"] + 2.0
    times["place_end"] = times["place_start"] + 2.0
    X_WG["place_start"] = X_WG["place"]
    X_WG["place_end"] = X_WG["place"]
    times["postplace"] = times["place_end"] + 2.0
    X_WG["postplace"] = X_WG["preplace"]

    return X_WG, times
