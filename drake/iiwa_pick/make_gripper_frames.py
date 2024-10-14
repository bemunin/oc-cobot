import os

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
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

    # frames

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
    p_Ginitial = np.array([0.4656, 0, 0.6793])
    X_G = {"initial": RigidTransform(R_Ginitial, p_Ginitial)}

    R_Oinitial = RotationMatrix()  # Identity
    p_Oinitial = np.array([0.82, -0.03, -0.046])

    R_Ogoal = RotationMatrix.MakeZRotation(np.pi / 2.0)
    p_Ogoal = p_Oinitial.copy() + np.array([-1.45, 0, 0.1])

    X_O = {
        "initial": RigidTransform(R_Oinitial, p_Oinitial),
        "goal": RigidTransform(R_Ogoal, p_Ogoal),
    }

    visualize_gripper_frames(X_G, X_O, meshcat)
