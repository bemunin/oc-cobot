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
    times["initial"] = 0.0

    return X_WG, times


def sketch_gripper(meshcat):
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

    X_G, _ = MakeGripperFrames(X_G, X_O)
    visualize_gripper_frames(X_G, X_O, meshcat)
