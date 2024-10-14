import numpy as np
from pydrake.all import (
    RigidTransform,
    RotationMatrix,
)


def visualize_gripper_frames():
    pass


def MakeGripperFrames():
    pass


def sketch_gripper():
    initial_gripper_rot = RotationMatrix(
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
    initial_gripper_pos = np.array([0.4656, 0, 0.6793])
    X_G = {"initial": RigidTransform(initial_gripper_rot, initial_gripper_pos)}
    print(X_G["initial"])
