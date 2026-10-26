#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 IMU 数据从自身坐标系旋转到车体坐标系（base_link）。

输入是 parse_bag.py 导出的 IMU 文本文件，格式（默认无表头，空格分隔）：
    timestamp acc_x acc_y acc_z gyro_x gyro_y gyro_z
    加速度单位 m/s^2，角速度单位 rad/s，第一列为时间戳（秒）。
若输入文件首行是表头（非数字），脚本会自动识别并跳过。

外参安装角定义：
    IMU 坐标系相对车体 base_link 的安装姿态，用 roll / pitch / yaw 三个角表示
    （ZYX 欧拉角，单位：度）。对应旋转矩阵 R = Rz(yaw)·Ry(pitch)·Rx(roll)，
    向量从 IMU 系转到车体系：v_base = R @ v_imu。
    （与 utils.RPY2mat 的约定一致；这里只做纯旋转，不考虑平移/杆臂。）

在下方 IMU_EXTRINSIC_DEG 中，以 IMU 的 topic 名（即文件名去掉 .txt 的部分）为
关键字配置安装角。关键字可以是完整 topic 名，也可以是能唯一区分该 IMU 的子串
（如 "livox" 会匹配所有 livox_imu_*.txt）。命中多个时取第一个匹配的配置。

输出文件在原文件名后缀加 _base_link，例如：
    gps_imu_raw.txt            -> gps_imu_raw_base_link.txt
    livox_imu_192_168_1_158.txt -> livox_imu_192_168_1_158_base_link.txt

用法示例：
    python3 imu_to_base_link.py --dir output/rosbag2_2026_08_20-09_02_24_parsed_res
    python3 imu_to_base_link.py --files gps_imu_raw.txt livox_imu_192_168_1_158.txt
    python3 imu_to_base_link.py --dir <dir> --suffix _body   # 自定义后缀
    python3 imu_to_base_link.py --files a.txt --roll 0 --pitch 0 --yaw 90  # 单个 IMU 统一外参
"""

import argparse
import math
from pathlib import Path

# ======================== 配置：IMU 安装外参 ========================
# 键：IMU 的 topic 名 / 文件名关键字；值：(roll, pitch, yaw)，单位：度（ZYX 欧拉角）。
# 也可以直接给 3x3 旋转矩阵（3 行 3 列的嵌套 list），表示 IMU -> base_link 的旋转。
# 示例：
#   "gps_imu_raw": (0.0, 0.0, 0.0),
#   "livox_imu":   (0.0, 0.0, 90.0),          # 绕 z 轴转 90 度
#   "some_imu":    [[0, -1, 0],
#                   [1,  0, 0],
#                   [0,  0, 1]],              # 等价于 yaw=90 度的旋转矩阵
IMU_EXTRINSIC_DEG = {
    # 在这里补充各 IMU 的安装角
    "gps_imu_raw": (-0.35, 4.34, -91.0),
    "livox_imu_192_168_1_158": (2.69, 60.46, 180.0),
    "livox_imu_192_168_1_181": (0.000000, 45.000000, 45.000000)
}
# ===================================================================


def rpy_to_matrix(roll_rad, pitch_rad, yaw_rad):
    """ZYX 欧拉角 -> 3x3 旋转矩阵 R = Rz(yaw)·Ry(pitch)·Rx(roll)。

    与 utils.RPY2mat 完全一致，返回 list[list[float]]，用于 v_base = R @ v_imu。
    """
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def mat_vec(R, v):
    """3x3 矩阵 R 乘 3 维向量 v。"""
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def transpose(R):
    return [[R[j][i] for j in range(3)] for i in range(3)]


def resolve_extrinsic(value):
    """把配置里的外参转成 3x3 旋转矩阵。

    接受 (roll, pitch, yaw) 三元组（度，ZYX 欧拉角）或 3x3 嵌套 list（旋转矩阵）。
    """
    if isinstance(value, (list, tuple)) and len(value) == 3:
        if all(isinstance(row, (list, tuple)) for row in value):
            # 3x3 旋转矩阵
            R = [[float(x) for x in row] for row in value]
            if any(len(row) != 3 for row in R):
                raise ValueError(f"旋转矩阵必须是 3x3：{value}")
            return R
        # (roll, pitch, yaw) 角度（度）
        roll, pitch, yaw = (float(x) for x in value)
        return rpy_to_matrix(math.radians(roll), math.radians(pitch), math.radians(yaw))
    raise ValueError(f"无法识别的外参配置：{value!r}（应为 (roll, pitch, yaw) 或 3x3 矩阵）")


def match_extrinsic(stem, config):
    """按文件名关键字匹配外参配置，返回 (关键字, 旋转矩阵)；无匹配返回 None。"""
    stem_l = stem.lower()
    for key, value in config.items():
        if key.lower() in stem_l:
            return key, resolve_extrinsic(value)
    return None


def transform_file(in_path, R, out_path):
    """按 R 旋转 acc / gyro，写一个新文件。返回处理的帧数。"""
    n = 0
    header = None
    with open(in_path, encoding="utf-8") as fin, \
            open(out_path, "w", encoding="utf-8") as fout:
        for line in fin:
            line = line.rstrip("\n")
            if not line.strip():
                continue
            parts = line.split()
            try:
                _ = float(parts[0])
            except ValueError:
                header = line  # 表头行
                continue
            if len(parts) < 7:
                continue  # 列数不足，跳过

            ts = parts[0]
            acc = [float(parts[1]), float(parts[2]), float(parts[3])]
            gyro = [float(parts[4]), float(parts[5]), float(parts[6])]

            acc_b = mat_vec(R, acc)
            gyro_b = mat_vec(R, gyro)

            vals = [acc_b[0], acc_b[1], acc_b[2], gyro_b[0], gyro_b[1], gyro_b[2]]
            fout.write(ts + " " + " ".join(repr(float(v)) for v in vals) + "\n")
            n += 1

    # 表头行放到最前面
    if header is not None:
        content = out_path.read_text(encoding="utf-8")
        out_path.write_text(header + "\n" + content, encoding="utf-8")
    return n


def main():
    ap = argparse.ArgumentParser(description="IMU 数据：自身坐标系 -> 车体坐标系")
    ap.add_argument("--dir", default=None, help="目录，自动处理其中的 *_imu*.txt 文件")
    ap.add_argument("--files", nargs="+", default=[], help="一个或多个 IMU 文件路径")
    ap.add_argument("--suffix", default="_base_link", help="输出文件名后缀（默认 _base_link）")
    ap.add_argument("--out", default=None, help="输出目录（默认与输入文件同目录）")
    ap.add_argument("--roll", type=float, default=None, help="统一外参 roll（度），覆盖配置")
    ap.add_argument("--pitch", type=float, default=None, help="统一外参 pitch（度），覆盖配置")
    ap.add_argument("--yaw", type=float, default=None, help="统一外参 yaw（度），覆盖配置")
    ap.add_argument("--invert", action="store_true",
                    help="对旋转矩阵取转置（即把给定外参当作 base_link -> IMU）")
    args = ap.parse_args()

    # 命令行统一外参（需同时给出 roll/pitch/yaw）
    uniform = [args.roll, args.pitch, args.yaw]
    if any(v is not None for v in uniform):
        if any(v is None for v in uniform):
            ap.error("--roll/--pitch/--yaw 必须同时给出")
        uniform_R = rpy_to_matrix(math.radians(args.roll),
                                  math.radians(args.pitch),
                                  math.radians(args.yaw))
    else:
        uniform_R = None

    # 收集输入文件
    files = [Path(f) for f in args.files]
    if args.dir:
        d = Path(args.dir)
        files += sorted(p for p in d.glob("*.txt")
                        if "imu" in p.stem.lower() and
                        not p.stem.endswith(args.suffix))

    if not files:
        ap.error("请用 --dir 或 --files 指定输入文件")

    out_dir = Path(args.out) if args.out else None
    total = 0
    for in_path in files:
        stem = in_path.stem

        if uniform_R is not None:
            R = uniform_R
            src = "(命令行外参)"
        else:
            hit = match_extrinsic(stem, IMU_EXTRINSIC_DEG)
            if hit is None:
                print(f"[跳过] {in_path.name}: 未在 IMU_EXTRINSIC_DEG 中配置外参", )
                continue
            src, R = hit

        if args.invert:
            R = transpose(R)

        out_path = in_path.with_name(stem + args.suffix + in_path.suffix)
        if out_dir is not None:
            out_path = out_dir / out_path.name

        n = transform_file(in_path, R, out_path)
        total += n
        print(f"[完成] {in_path.name} -> {out_path.name} ({n} 帧)  外参来自: {src}")

    print(f"\n共处理 {total} 帧 IMU 数据。")


if __name__ == "__main__":
    main()
