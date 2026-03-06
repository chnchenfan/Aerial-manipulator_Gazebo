#!/usr/bin/env python3
"""
Quick helper to compute parent->child relative pose (xyz/rpy) for xacro
based on absolute poses (same reference frame, e.g. model frame from SDF).
Fill in the examples at the bottom and run the script.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R


def get_matrix(xyz, rpy):
    """Build a 4x4 transform matrix from xyz and rpy (rad)."""
    r = R.from_euler("xyz", rpy)
    matrix = np.eye(4)
    matrix[:3, :3] = r.as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def calc_relative(parent_xyz, parent_rpy, child_xyz, child_rpy):
    """Return child pose w.r.t. parent."""
    t_parent = get_matrix(parent_xyz, parent_rpy)
    t_child = get_matrix(child_xyz, child_rpy)
    t_relative = np.linalg.inv(t_parent) @ t_child
    rel_xyz = t_relative[:3, 3]
    rel_rpy = R.from_matrix(t_relative[:3, :3]).as_euler("xyz")
    return rel_xyz, rel_rpy


def print_relative(name, parent_xyz, parent_rpy, child_xyz, child_rpy):
    rel_xyz, rel_rpy = calc_relative(parent_xyz, parent_rpy, child_xyz, child_rpy)
    print("-" * 56)
    print(f"{name}")
    print(f"Parent xyz={parent_xyz}, rpy={parent_rpy}")
    print(f"Child  xyz={child_xyz}, rpy={child_rpy}")
    print(f"Relative XYZ: {rel_xyz[0]:.5f} {rel_xyz[1]:.5f} {rel_xyz[2]:.5f}")
    print(f"Relative RPY: {rel_rpy[0]:.5f} {rel_rpy[1]:.5f} {rel_rpy[2]:.5f}")


if __name__ == "__main__":
    # Examples using absolute poses from SDF (model frame) for arm joints.
    # If你调整了SDF pose，记得同步更新这里。
    examples = [
        dict(
            name="arm_joint1",  # parent: base_link, child: arm_link1
            parent_xyz=[0.0, 0.0, 0.0],
            parent_rpy=[0.0, 0.0, 0.0],
            child_xyz=[0.0, 0.0, 0.1745],
            child_rpy=[3.14159, 0.0, 3.14159],
        ),
        dict(
            name="arm_joint2",  # parent: arm_link1, child: arm_link2
            parent_xyz=[0.0, 0.0, 0.1745],
            parent_rpy=[3.14159, 0.0, 3.14159],
            child_xyz=[0.0, 0.0, 0.0425],
            child_rpy=[-1.57079, 0.0, -3.14159],
        ),
        dict(
            name="arm_joint3",  # parent: arm_link2, child: arm_link3
            parent_xyz=[0.0, 0.0, 0.0425],
            parent_rpy=[-1.57079, 0.0, -3.14159],
            child_xyz=[-0.096144, -1e-06, 0.015],
            child_rpy=[-1.57079, 0.27859, -3.14159],
        ),
        dict(
            name="arm_joint4",  # parent: arm_link3, child: arm_link4
            parent_xyz=[-0.096144, -1e-06, 0.015],
            parent_rpy=[-1.57079, 0.27859, -3.14159],
            child_xyz=[-0.184644, -1e-06, 0.015],
            child_rpy=[-3.14159, 0.0, -3.14159],
        ),
        dict(
            name="left_gripper_joint",  # parent: arm_link4, child: left_gripper_link
            parent_xyz=[-0.184644, -1e-06, 0.015],
            parent_rpy=[-3.14159, 0.0, -3.14159],
            child_xyz=[-0.262394, -0.005808, 0.0325],
            child_rpy=[0.0, 0.0, -3.14159],
        ),
        dict(
            name="right_gripper_joint",  # parent: arm_link4, child: right_gripper_link
            parent_xyz=[-0.184644, -1e-06, 0.015],
            parent_rpy=[-3.14159, 0.0, -3.14159],
            child_xyz=[-0.262394, 0.005942, 0.032375],
            child_rpy=[0.0, 0.0, -3.14159],
        ),
    ]

    for ex in examples:
        print_relative(
            ex["name"],
            ex["parent_xyz"],
            ex["parent_rpy"],
            ex["child_xyz"],
            ex["child_rpy"],
        )
