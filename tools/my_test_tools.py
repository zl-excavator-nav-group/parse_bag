import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import utils


def xyzrpy_deg_to_se3(xyz, rpy_deg):
    """
    Build an SE(3) transform from translation xyz and RPY angles in degrees.

    Args:
        xyz: array-like, [x, y, z]
        rpy_deg: array-like, [roll, pitch, yaw] in degrees

    Returns:
        T: 4x4 homogeneous transform
        T_inv: inverse of T
    """
    xyz = np.asarray(xyz, dtype=np.float64).reshape(3)
    rpy_deg = np.asarray(rpy_deg, dtype=np.float64).reshape(3)
    rpy_rad = np.radians(rpy_deg)

    R = np.asarray(utils.RPY2mat(rpy_rad[0], rpy_rad[1], rpy_rad[2]),
                   dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = xyz

    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ xyz

    return T, T_inv


if __name__ == "__main__":
    # T_ins_base
    xyz = np.array([0.65, -0.052, 0.0])  # x, y, z
    rpy_deg = np.array([-1.3, -0.5, 1.0])  # roll, pitch, yaw
    T_ins_base, T_base_ins = xyzrpy_deg_to_se3(xyz, rpy_deg)
    # print(f"xyz = {xyz}")
    # print(f"rpy_deg = {rpy_deg}")
    # print(f"T_ins_base =\n{T_ins_base}")
    print(f"T_base_ins =\n{T_base_ins}")
    print(f"xyz_base_ins = {T_base_ins[:3, 3]}")
    print(f"rpy_deg_base_ins = {np.degrees(utils.mat2RPY(T_base_ins[:3, :3]))}")


    # T_ins_lidar
    xyz = np.array([-0.912417, 0.0220508, 2.16])  # x, y, z
    rpy_deg = np.array([4.4408, 61.2352, -177.107])  # roll, pitch, yaw
    T_ins_lidar, T_lidar_ins = xyzrpy_deg_to_se3(xyz, rpy_deg)
    print(f"xyz = {xyz}")
    print(f"rpy_deg = {rpy_deg}")
    print(f"T_ins_lidar =\n{T_ins_lidar}")
    print(f"T_lidar_ins =\n{T_lidar_ins}")

    T_base_lidar = T_base_ins @ T_ins_lidar
    print(f"T_base_lidar =\n{T_base_lidar}")
    rpy_deg_base_lidar = np.degrees(utils.mat2RPY(T_base_lidar[:3, :3]))
    print(f"xyz_base_lidar = {T_base_lidar[:3, 3]}")
    print(f"rpy_deg_base_lidar = {rpy_deg_base_lidar}")

    xyz = np.array([0.325, -1.093, 0])  # x, y, z
    rpy_deg = np.array([0, 0, 2.23])  # roll, pitch, yaw
    T_rtk_base, T_base_rtk = xyzrpy_deg_to_se3(xyz, rpy_deg)
    print(f"xyz_base_rtk = {T_base_rtk[:3, 3]}")
    print(f"rpy_deg_base_rtk = {np.degrees(utils.mat2RPY(T_base_rtk[:3, :3]))}")
    print(f"T_base_rtk =\n{T_base_rtk}")


    # T_rtk_base
    # xyz = np.array([0.325, -1.093, 0.0])  # x, y, z
    # rpy_deg = np.array([0, 0, 2.0])  # roll, pitch, yaw
    # T_rtk_base, T_base_rtk = xyzrpy_deg_to_se3(xyz, rpy_deg)
    # # print(f"xyz = {xyz}")
    # # print(f"rpy_deg = {rpy_deg}")
    # # print(f"T_ins_base =\n{T_ins_base}")
    # print(f"T_base_rtk =\n{T_base_rtk}")
    # print(f"xyz_base_rtk = {T_base_rtk[:3, 3]}")
    # print(f"rpy_deg_base_rtk = {np.degrees(utils.mat2RPY(T_base_rtk[:3, :3]))}")

    # 2t T_rtk_base
    xyz = np.array([0.293, 0.787, 0.0])  # x, y, z
    rpy_deg = np.array([0, 0, 90.0])  # roll, pitch, yaw
    T_rtk_base, T_base_rtk = xyzrpy_deg_to_se3(xyz, rpy_deg)
    # print(f"xyz = {xyz}")
    # print(f"rpy_deg = {rpy_deg}")
    # print(f"T_ins_base =\n{T_ins_base}")
    print(f"T_base_rtk =\n{T_base_rtk}")
    print(f"xyz_base_rtk = {T_base_rtk[:3, 3]}")
    print(f"rpy_deg_base_rtk = {np.degrees(utils.mat2RPY(T_base_rtk[:3, :3]))}")


    # T_base_lidar
    xyz = np.array([-0.07936, 0.390, 0.97607272494944763])  # x, y, z
    rpy_deg = np.array([5.3837508038443884, 45.353132478525112, 85.24])  # roll, pitch, yaw
    T_base_lidar, T_lidar_base = xyzrpy_deg_to_se3(xyz, rpy_deg)
    # print(f"xyz = {xyz}")
    # print(f"rpy_deg = {rpy_deg}")
    # print(f"T_ins_base =\n{T_ins_base}")
    print(f"T_base_lidar =\n{T_base_lidar}")
    print(f"xyz_base_lidar = {T_base_lidar[:3, 3]}")
    print(f"rpy_deg_base_lidar = {np.degrees(utils.mat2RPY(T_base_lidar[:3, :3]))}")

    









