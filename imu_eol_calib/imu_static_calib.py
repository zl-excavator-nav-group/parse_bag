#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 静态标定（EOL / 出厂标定）工具。

输入为 parse_bag.py 解析出的 IMU 原始数据文件（空格分隔，无表头）：

    timestamp acc_x acc_y acc_z gyro_x gyro_y gyro_z

其中 timestamp 单位为秒（小数部分为纳秒），acc 单位 m/s^2，gyro 单位 rad/s。

算法逐行移植自 C++ 实现：
    holm_egomotion/src/calibration/factory/imu/vcs_imu_fac.cpp
        -> VcsImuFacCalib::CalibProcess
输出文件格式对齐：
    holm_egomotion/app/test_3dodo/data_adapter.cpp
        -> DataAdapter::SaveImuEolRes

基本流程：
    1. 按指定时间范围截取一段（应为车辆静止）的 IMU 数据；
    2. 统计 acc / gyro 的均值与标准差、帧间时间戳统计、丢帧率；
    3. 由 acc 均值（即重力方向）用 Gram-Schmidt 解算 roll / pitch；
       yaw 无法由重力观测得到，用配置中 qCI 的 yaw 强制覆盖；
    4. 由重力残差反解 acc bias，gyro 均值即 gyro bias；
    5. 输出 imu_calibration.json / imu_calibration_checksum.json / imu_param.json。

用法：
    python3 imu_static_calib.py config.yaml
    python3 imu_static_calib.py config.yaml --scan          # 搜索静止窗口
    python3 imu_static_calib.py config.yaml --start 1787218160 --end 1787218180
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from enum import IntFlag
from pathlib import Path

import numpy as np
import yaml

# 与 C++ vcs_imu_fac.cpp 中的 #define GRAVITY 保持一致（注意不是 9.80665）
GRAVITY = 9.81

# 相对时间偏移的判定阈值：小于该值的时间戳按“相对首帧的秒偏移”解释
REL_TS_THRESHOLD = 1e6


class ImuCalibErrorCode(IntFlag):
    """标定错误码位标志，取值与 C++ ImuCalibErrorCode 枚举一一对应。"""

    CALIB_NO_ERROR = 0
    CALIB_ERROR = 1 << 0
    CALIB_NOT_INPUT_DATA = 1 << 1
    CALIB_EOL_STILL_DATA_COLLECTING = 1 << 2
    CALIB_EOL_PITCH_OUTOF_RANGE = 1 << 3
    CALIB_EOL_ROLL_OUTOF_RANGE = 1 << 4
    CALIB_EOL_IMU_PARAM_SAVE_FAILED = 1 << 5
    CALIB_EOL_TILTVERIFY_BEGIN_NOT_STATIC = 1 << 6
    CALIB_EOL_TILTVERIFY_END_NOT_STATIC = 1 << 7
    CALIB_EOL_TILTVERIFY_NOT_CAL_FRONT = 1 << 8
    CALIB_EOL_IMU_ACC_EXCEPTION = 1 << 9
    CALIB_EOL_IMU_ACC_STD_EXCEPTION = 1 << 10
    CALIB_EOL_IMU_GYRO_EXCEPTION = 1 << 11
    CALIB_EOL_IMU_GYRO_STD_EXCEPTION = 1 << 12
    CALIB_EOL_IMU_TIMESTAMP_EXCEPTION = 1 << 13
    CALIB_EOL_IMU_TIMESTAMP_STD_EXCEPTION = 1 << 14
    CALIB_EOL_IMU_DROP_FRAME_EXCEPTION = 1 << 15


def describe_error_code(code: int) -> str:
    """把错误码位标志展开成可读字符串。"""
    if code == 0:
        return "CALIB_NO_ERROR"
    names = [f.name for f in ImuCalibErrorCode if f.value != 0 and code & f.value]
    return " | ".join(names) if names else f"UNKNOWN(0x{code:04X})"


# ---------------------------------------------------------------------------
# 旋转数学：逐行移植 C++，不复用 utils.py，以保证万向锁分支与角度归一化完全一致
# ---------------------------------------------------------------------------


def normalize_angle(z: float) -> float:
    """将角度归一化到 [-pi, pi]。对应 C++ NormalizeAngle。"""
    return math.atan2(math.sin(z), math.cos(z))


def matrix2euler(dcm: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> [roll, pitch, yaw]（ZYX 顺序，弧度）。对应 C++ matrix2euler。"""
    euler = np.zeros(3)
    euler[1] = math.atan(
        -dcm[2, 0] / math.sqrt(dcm[2, 1] * dcm[2, 1] + dcm[2, 2] * dcm[2, 2])
    )
    if dcm[2, 0] <= -0.999:
        euler[0] = 0.0
        euler[2] = math.atan2(dcm[1, 2] - dcm[0, 1], dcm[0, 2] + dcm[1, 1])
    elif dcm[2, 0] >= 0.999:
        euler[0] = 0.0
        euler[2] = math.pi + math.atan2(dcm[1, 2] + dcm[0, 1], dcm[0, 2] - dcm[1, 1])
    else:
        euler[0] = math.atan2(dcm[2, 1], dcm[2, 2])
        euler[2] = math.atan2(dcm[1, 0], dcm[0, 0])
    return np.array([normalize_angle(a) for a in euler])


def euler2matrix(euler: np.ndarray) -> np.ndarray:
    """[roll, pitch, yaw] -> 旋转矩阵 Rz(yaw) * Ry(pitch) * Rx(roll)。对应 C++ euler2matrix。"""
    roll, pitch, yaw = float(euler[0]), float(euler[1]), float(euler[2])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def gravity_solution(acc: np.ndarray) -> np.ndarray:
    """由重力方向解算姿态，返回 R_G_I（IMU 系 -> 全局系）。对应 C++ GravitySolution。"""
    z_axis = acc / np.linalg.norm(acc)
    unit_x = np.array([1.0, 0.0, 0.0])
    x_axis = unit_x - z_axis * float(z_axis @ unit_x)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    # R_I_G 的列分别为 x/y/z 轴，即全局系 -> IMU 系
    r_i_g = np.column_stack([x_axis, y_axis, z_axis])
    return r_i_g.T


def rotmat_to_quat(mat: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 四元数 [x, y, z, w]，与 Eigen::Quaterniond(Matrix3d) 算法一致。"""
    coeffs = np.zeros(4)  # [x, y, z, w]
    t = float(np.trace(mat))
    if t > 0.0:
        t = math.sqrt(t + 1.0)
        coeffs[3] = 0.5 * t
        t = 0.5 / t
        coeffs[0] = (mat[2, 1] - mat[1, 2]) * t
        coeffs[1] = (mat[0, 2] - mat[2, 0]) * t
        coeffs[2] = (mat[1, 0] - mat[0, 1]) * t
    else:
        i = 0
        if mat[1, 1] > mat[0, 0]:
            i = 1
        if mat[2, 2] > mat[i, i]:
            i = 2
        j = (i + 1) % 3
        k = (j + 1) % 3
        t = math.sqrt(mat[i, i] - mat[j, j] - mat[k, k] + 1.0)
        coeffs[i] = 0.5 * t
        t = 0.5 / t
        coeffs[3] = (mat[k, j] - mat[j, k]) * t
        coeffs[j] = (mat[j, i] + mat[i, j]) * t
        coeffs[k] = (mat[k, i] + mat[i, k]) * t
    return coeffs


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    """四元数 [w, x, y, z] -> 旋转矩阵（配置中 qCI 的顺序即 w,x,y,z）。"""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        raise ValueError("qCI 为零四元数")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE：init 0xFFFF，poly 0x1021，不反转，无末异或。对应 C++ CRC16_CCITT。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        crc &= 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# 参数与结果
# ---------------------------------------------------------------------------


@dataclass
class CalibParams:
    """镜像 C++ VcsImuFacParam 中标定实际使用到的字段。"""

    imu_freq: float = 100.0
    duration_time_ms: float = 15000.0
    rpy_thred: float = 3.0
    standard_roll: float = 0.0
    standard_pitch: float = 0.0
    yaw_vi: float = 0.0  # 弧度，来自 qCI
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # 以下阈值仅在 enable_check 为 True 时生效（C++ 中 enable_check_ 默认 false）
    enable_check: bool = False
    acc_norm_thred: float = 0.4
    acc_std_thred: float = 0.1
    gyro_norm_thred: float = 0.17
    gyro_std_thred: float = 0.02
    drop_frame_rate_thred: float = 0.05
    time_stamp_mean_thred: float = 2.0
    time_stamp_std_thred: float = 5.0


@dataclass
class CalibResult:
    """标定结果 + 数据统计。"""

    error_code: int = int(ImuCalibErrorCode.CALIB_NOT_INPUT_DATA | ImuCalibErrorCode.CALIB_ERROR)
    calib_finished: bool = False
    # 外参 / 内参
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [roll,pitch,yaw] 弧度
    rot: np.ndarray = field(default_factory=lambda: np.eye(3))
    quat_xyzw: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acc_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # 重力解算得到的原始 rpy（度），yaw 覆盖前，超差检查用它
    rpy_deg_raw: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # 数据统计
    acc_mean: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acc_std: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_mean: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro_std: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ts_diff_mean: float = 0.0
    ts_diff_std: float = 0.0
    ts_diff_min: float = 0.0
    ts_diff_max: float = 0.0
    drop_frame_rate: float = 0.0
    sample_count: int = 0
    dropped_count: int = 0
    t_start: float = 0.0
    t_end: float = 0.0


# ---------------------------------------------------------------------------
# 数据读取
# ---------------------------------------------------------------------------


def read_imu_file(path: Path) -> np.ndarray:
    """读取 7 列 IMU 文件，返回 (N, 7) 数组。自动跳过可能存在的表头行。"""
    if not path.is_file():
        raise SystemExit(f"IMU 数据文件不存在: {path}")
    try:
        data = np.loadtxt(path, usecols=range(7))
    except ValueError:
        data = np.loadtxt(path, usecols=range(7), skiprows=1)
    data = np.atleast_2d(data)
    if data.shape[0] < 2:
        raise SystemExit(f"IMU 数据不足（{data.shape[0]} 帧）: {path}")
    return data


def resolve_timestamp(value: float, t0: float) -> float:
    """支持绝对 epoch 秒与相对首帧秒偏移两种写法。"""
    if 0.0 < value < REL_TS_THRESHOLD:
        return t0 + value
    return value


def select_range(data: np.ndarray, start: float, end: float) -> np.ndarray:
    """按时间范围截取数据。start/end 为 0 表示不限。"""
    t0 = float(data[0, 0])
    mask = np.ones(data.shape[0], dtype=bool)
    if start:
        mask &= data[:, 0] >= resolve_timestamp(start, t0)
    if end:
        mask &= data[:, 0] <= resolve_timestamp(end, t0)
    return data[mask]


def dedup_increasing(data: np.ndarray) -> tuple[np.ndarray, int]:
    """丢弃时间戳非递增的帧，对应 C++ AdaptImuV2Data 中的去重逻辑。"""
    keep = np.ones(data.shape[0], dtype=bool)
    last = -np.inf
    for i, ts in enumerate(data[:, 0]):
        if ts <= last:
            keep[i] = False
        else:
            last = ts
    return data[keep], int((~keep).sum())


# ---------------------------------------------------------------------------
# 标定主流程
# ---------------------------------------------------------------------------


def calib_process(data: np.ndarray, params: CalibParams) -> CalibResult:
    """批量版 VcsImuFacCalib::CalibProcess。data 为已截取好的 (N, 7) 数组。"""
    result = CalibResult()

    data, dropped = dedup_increasing(data)
    result.dropped_count = dropped
    if data.shape[0] < 3:
        result.error_code = int(
            ImuCalibErrorCode.CALIB_NOT_INPUT_DATA | ImuCalibErrorCode.CALIB_ERROR
        )
        return result

    ts = data[:, 0]
    result.t_start = float(ts[0])
    result.t_end = float(ts[-1])

    # C++ 中首帧只用于初始化 init_ts / last_imu_meas_，不计入 acc/gyro 统计，这里保持一致
    acc = data[1:, 1:4]
    gyro = data[1:, 4:7]
    result.sample_count = int(acc.shape[0])

    # 帧间时间戳统计（单位 ms）。C++ 源数据为整数毫秒，这里保留浮点以利用更高的时间分辨率
    ts_diff = np.diff(ts) * 1000.0
    nominal_ts_diff = 1000.0 / params.imu_freq
    result.ts_diff_mean = float(ts_diff.mean())
    result.ts_diff_std = float(ts_diff.std(ddof=1)) if ts_diff.size >= 2 else 0.0
    result.ts_diff_min = float(ts_diff.min())
    result.ts_diff_max = float(ts_diff.max())
    frame_loss_cnt = int((ts_diff > nominal_ts_diff * 1.5).sum())
    result.drop_frame_rate = float(frame_loss_cnt) / float(ts_diff.size)

    # acc / gyro 统计。C++ 用 Welford 在线算法，GetStd() 使用 n-1，与 np.std(ddof=1) 等价
    result.acc_mean = acc.mean(axis=0)
    result.acc_std = acc.std(axis=0, ddof=1)
    result.gyro_mean = gyro.mean(axis=0)
    result.gyro_std = gyro.std(axis=0, ddof=1)

    # 数据时长门限
    span_ms = (result.t_end - result.t_start) * 1000.0
    if span_ms < params.duration_time_ms:
        result.error_code = int(
            ImuCalibErrorCode.CALIB_EOL_STILL_DATA_COLLECTING | ImuCalibErrorCode.CALIB_ERROR
        )
        return result

    error_code = 0

    # 数据质量检查（对齐 C++ enable_check_，默认关闭）
    if params.enable_check:
        acc_norm = float(np.linalg.norm(result.acc_mean))
        if acc_norm > GRAVITY + params.acc_norm_thred or acc_norm < GRAVITY - params.acc_norm_thred:
            error_code |= int(ImuCalibErrorCode.CALIB_EOL_IMU_ACC_EXCEPTION)
        if np.linalg.norm(result.acc_std) > params.acc_std_thred:
            error_code |= int(ImuCalibErrorCode.CALIB_EOL_IMU_ACC_STD_EXCEPTION)
        if np.linalg.norm(result.gyro_mean) > params.gyro_norm_thred:
            error_code |= int(ImuCalibErrorCode.CALIB_EOL_IMU_GYRO_EXCEPTION)
        if np.linalg.norm(result.gyro_std) > params.gyro_std_thred:
            error_code |= int(ImuCalibErrorCode.CALIB_EOL_IMU_GYRO_STD_EXCEPTION)
        if result.drop_frame_rate > params.drop_frame_rate_thred:
            error_code |= int(ImuCalibErrorCode.CALIB_EOL_IMU_DROP_FRAME_EXCEPTION)
        if abs(result.ts_diff_mean - nominal_ts_diff) > params.time_stamp_mean_thred:
            error_code |= int(ImuCalibErrorCode.CALIB_EOL_IMU_TIMESTAMP_EXCEPTION)
        if result.ts_diff_std > params.time_stamp_std_thred:
            error_code |= int(ImuCalibErrorCode.CALIB_EOL_IMU_TIMESTAMP_STD_EXCEPTION)
        if error_code != 0:
            result.error_code = error_code | int(ImuCalibErrorCode.CALIB_ERROR)
            return result

    # 重力解算 roll / pitch，yaw 由配置强制覆盖
    rot = gravity_solution(result.acc_mean)
    rpy = matrix2euler(rot)
    result.rpy_deg_raw = np.degrees(rpy)
    rpy = rpy.copy()
    rpy[2] = params.yaw_vi
    rot = euler2matrix(rpy)

    result.rot = rot
    result.rpy = rpy
    result.quat_xyzw = rotmat_to_quat(rot)
    result.translation = np.asarray(params.translation, dtype=float)

    # acc bias：把 acc 均值旋到全局系扣掉重力，再旋回 IMU 系
    gravity_mean = rot @ result.acc_mean
    gravity_mean = gravity_mean.copy()
    gravity_mean[2] -= GRAVITY
    result.acc_bias = np.linalg.inv(rot) @ gravity_mean
    result.gyro_bias = result.gyro_mean
    result.calib_finished = True

    # roll / pitch 超差检查（C++ 中此检查不受 enable_check_ 控制，始终执行）
    if abs(result.rpy_deg_raw[1] - params.standard_pitch) > params.rpy_thred:
        error_code |= int(ImuCalibErrorCode.CALIB_EOL_PITCH_OUTOF_RANGE)
    if abs(result.rpy_deg_raw[0] - params.standard_roll) > params.rpy_thred:
        error_code |= int(ImuCalibErrorCode.CALIB_EOL_ROLL_OUTOF_RANGE)
    if error_code != 0:
        error_code |= int(ImuCalibErrorCode.CALIB_ERROR)

    result.error_code = error_code
    return result


# ---------------------------------------------------------------------------
# 结果输出
# ---------------------------------------------------------------------------


def _dump_json(obj: dict) -> str:
    """与 nlohmann::json 的 dump(4) 对齐：键名字典序、4 空格缩进、结尾无换行。"""
    return json.dumps(obj, indent=4, sort_keys=True, ensure_ascii=False)


def save_results(result: CalibResult, out_dir: Path) -> list[Path]:
    """写出三个 JSON 文件，格式对齐 C++ DataAdapter::SaveImuEolRes。"""
    out_dir.mkdir(parents=True, exist_ok=True)

    imu_json = {
        "type": "imu",
        "already_rw": 0,
        "calib_case": 4,
        "calib_status": 1,
        "yaw_errcode": 0,
        "calib_errCode": 0,
        "euler_angle_ci_x": float(result.rpy[0]),
        "euler_angle_ci_y": float(result.rpy[1]),
        "euler_angle_ci_z": float(result.rpy[2]),
        "quaternion_ci_x": float(result.quat_xyzw[0]),
        "quaternion_ci_y": float(result.quat_xyzw[1]),
        "quaternion_ci_z": float(result.quat_xyzw[2]),
        "quaternion_ci_w": float(result.quat_xyzw[3]),
        "tci_x": float(result.translation[0]),
        "tci_y": float(result.translation[1]),
        "tci_z": float(result.translation[2]),
        "acc_bias_x": float(result.acc_bias[0]),
        "acc_bias_y": float(result.acc_bias[1]),
        "acc_bias_z": float(result.acc_bias[2]),
        "gyro_bias_x": float(result.gyro_bias[0]),
        "gyro_bias_y": float(result.gyro_bias[1]),
        "gyro_bias_z": float(result.gyro_bias[2]),
    }
    imu_json_str = _dump_json(imu_json)
    calib_path = out_dir / "imu_calibration.json"
    calib_path.write_text(imu_json_str, encoding="utf-8")

    # CRC 校验文件：校验和算在上面那份 dump 字符串（即写盘的完全相同字节）上
    crc_json = {
        "checksum": crc16_ccitt(imu_json_str.encode("utf-8")),
        "algorithm": "crc16",
        "generated_at": int(time.time() * 1000),
    }
    crc_path = out_dir / "imu_calibration_checksum.json"
    crc_path.write_text(_dump_json(crc_json), encoding="utf-8")

    # 精简版参数文件
    param_json = {
        "type": "imu",
        "euler_angle_ci_z": float(result.rpy[2]),
        "tci_x": float(result.translation[0]),
        "tci_y": float(result.translation[1]),
        "tci_z": float(result.translation[2]),
    }
    param_path = out_dir / "imu_param.json"
    param_path.write_text(_dump_json(param_json), encoding="utf-8")

    return [calib_path, crc_path, param_path]


def _fmt_vec(v: np.ndarray, width: int = 12, prec: int = 6) -> str:
    return "  ".join(f"{float(x):>{width}.{prec}f}" for x in v)


def print_report(result: CalibResult, params: CalibParams) -> None:
    """打印控制台标定报告。"""
    span = result.t_end - result.t_start
    print("=" * 68)
    print("IMU 静态标定报告")
    print("=" * 68)
    print(f"  时间范围      : {result.t_start:.6f} ~ {result.t_end:.6f}  ({span:.3f} s)")
    print(f"  参与统计帧数  : {result.sample_count}"
          f"（时间戳非递增丢弃 {result.dropped_count} 帧）")
    print(f"  帧间隔 ms     : mean={result.ts_diff_mean:.3f}  std={result.ts_diff_std:.3f}  "
          f"min={result.ts_diff_min:.3f}  max={result.ts_diff_max:.3f}")
    print(f"  丢帧率        : {result.drop_frame_rate:.4f}"
          f"（阈值 {params.drop_frame_rate_thred}，判定间隔 > {1.5 * 1000.0 / params.imu_freq:.2f} ms）")
    print("-" * 68)
    print(f"  acc  mean     : {_fmt_vec(result.acc_mean)}   |acc|={np.linalg.norm(result.acc_mean):.6f} m/s^2")
    print(f"  acc  std      : {_fmt_vec(result.acc_std)}   |std|={np.linalg.norm(result.acc_std):.6f}")
    print(f"  gyro mean     : {_fmt_vec(result.gyro_mean, prec=9)}   |gyro|={np.linalg.norm(result.gyro_mean):.9f} rad/s")
    print(f"  gyro std      : {_fmt_vec(result.gyro_std, prec=9)}   |std|={np.linalg.norm(result.gyro_std):.9f}")
    print("-" * 68)
    if result.calib_finished:
        print(f"  重力解算 rpy  : roll={result.rpy_deg_raw[0]:.6f}  pitch={result.rpy_deg_raw[1]:.6f} (deg)")
        print(f"  标定外参 rpy  : roll={math.degrees(result.rpy[0]):.6f}  "
              f"pitch={math.degrees(result.rpy[1]):.6f}  "
              f"yaw={math.degrees(result.rpy[2]):.6f} (deg，yaw 来自 qCI)")
        print(f"  四元数 xyzw   : {_fmt_vec(result.quat_xyzw, prec=9)}")
        print(f"  平移 tci      : {_fmt_vec(result.translation)}  (m)")
        print(f"  acc  bias     : {_fmt_vec(result.acc_bias, prec=9)}  (m/s^2)")
        print(f"  gyro bias     : {_fmt_vec(result.gyro_bias, prec=9)}  (rad/s)")
    else:
        print("  标定未完成，无外参 / 内参结果")
    print("-" * 68)
    print(f"  error_code    : {result.error_code}  ({describe_error_code(result.error_code)})")
    print("=" * 68)


# ---------------------------------------------------------------------------
# 静止窗口搜索（C++ 无此功能，用于辅助挑选标定时间段）
# ---------------------------------------------------------------------------


def scan_static_windows(data: np.ndarray, params: CalibParams, topk: int, step_s: float = 1.0) -> None:
    """按 duration_time_ms 长度滑窗扫描全程，按 |gyro_std| 升序列出最静止的若干候选窗口。"""
    ts = data[:, 0]
    win_s = params.duration_time_ms / 1000.0
    t_begin, t_final = float(ts[0]), float(ts[-1])
    if t_final - t_begin < win_s:
        print(f"数据总时长 {t_final - t_begin:.3f} s 小于窗口长度 {win_s:.3f} s，无法扫描")
        return

    candidates = []
    t = t_begin
    while t + win_s <= t_final:
        lo, hi = np.searchsorted(ts, [t, t + win_s])
        if hi - lo >= 3:
            acc = data[lo:hi, 1:4]
            gyro = data[lo:hi, 4:7]
            candidates.append(
                (
                    float(np.linalg.norm(gyro.std(axis=0, ddof=1))),
                    float(np.linalg.norm(acc.std(axis=0, ddof=1))),
                    float(np.linalg.norm(acc.mean(axis=0))),
                    t,
                    t + win_s,
                    int(hi - lo),
                )
            )
        t += step_s

    if not candidates:
        print("未找到有效窗口")
        return

    candidates.sort(key=lambda c: c[0])
    print("=" * 96)
    print(f"静止窗口候选（窗口长度 {win_s:.1f} s，步长 {step_s:.1f} s，共 {len(candidates)} 个窗口，按 |gyro_std| 升序）")
    print("=" * 96)
    print(f"{'#':>3}  {'|gyro_std|':>12}  {'|acc_std|':>12}  {'|acc_mean|':>12}  "
          f"{'帧数':>6}  {'start_timestamp':>18}  {'end_timestamp':>18}")
    print("-" * 96)
    for idx, (g_std, a_std, a_mean, t0, t1, n) in enumerate(candidates[:topk], 1):
        print(f"{idx:>3}  {g_std:>12.6f}  {a_std:>12.6f}  {a_mean:>12.6f}  "
              f"{n:>6}  {t0:>18.6f}  {t1:>18.6f}")
    print("-" * 96)
    best = candidates[0]
    print("可直接填入配置：")
    print(f"  start_timestamp: {best[3]:.6f}")
    print(f"  end_timestamp:   {best[4]:.6f}")
    print("=" * 96)


# ---------------------------------------------------------------------------
# 配置与入口
# ---------------------------------------------------------------------------


def load_config(path: Path) -> dict:
    if not path.is_file():
        raise SystemExit(f"配置文件不存在: {path}")
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    if not isinstance(cfg, dict):
        raise SystemExit(f"配置文件格式错误（应为 YAML 映射）: {path}")
    return cfg


def build_params(cfg: dict, enable_check_override: bool) -> CalibParams:
    """由配置构造标定参数。yaw 取自 qCI（与 C++ 一致，qCI 的 roll/pitch 被忽略）。"""
    q_ci = np.asarray(cfg.get("qCI", [1.0, 0.0, 0.0, 0.0]), dtype=float)
    if q_ci.size != 4:
        raise SystemExit("qCI 必须是长度为 4 的数组 [w, x, y, z]")
    yaw_vi = float(matrix2euler(quat_wxyz_to_matrix(q_ci))[2])

    t_ci = np.asarray(cfg.get("tCI", [0.0, 0.0, 0.0]), dtype=float)
    if t_ci.size != 3:
        raise SystemExit("tCI 必须是长度为 3 的数组 [x, y, z]")

    return CalibParams(
        imu_freq=float(cfg.get("imu_freq", 100.0)),
        duration_time_ms=float(cfg.get("duration_time_ms", 15000)),
        rpy_thred=float(cfg.get("rpy_thred", 3.0)),
        standard_roll=float(cfg.get("standard_roll", 0.0)),
        standard_pitch=float(cfg.get("standard_pitch", 0.0)),
        yaw_vi=yaw_vi,
        translation=t_ci,
        enable_check=bool(cfg.get("enable_check", False)) or enable_check_override,
        acc_norm_thred=float(cfg.get("acc_norm_thred", 0.4)),
        acc_std_thred=float(cfg.get("acc_std_thred", 0.1)),
        gyro_norm_thred=float(cfg.get("gyro_norm_thred", 0.17)),
        gyro_std_thred=float(cfg.get("gyro_std_thred", 0.02)),
        drop_frame_rate_thred=float(cfg.get("drop_frame_rate_thred", 0.05)),
        time_stamp_mean_thred=float(cfg.get("time_stamp_mean_thred", 2.0)),
        time_stamp_std_thred=float(cfg.get("time_stamp_std_thred", 5.0)),
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="IMU 静态标定（EOL），算法对齐 C++ VcsImuFacCalib::CalibProcess",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("config", type=Path, help="YAML 配置文件路径")
    parser.add_argument("--imu-file", type=Path, default=None, help="覆盖配置中的 imu_file")
    parser.add_argument("--start", type=float, default=None,
                        help="覆盖 start_timestamp（绝对 epoch 秒，或 <1e6 时按相对首帧秒偏移）")
    parser.add_argument("--end", type=float, default=None, help="覆盖 end_timestamp")
    parser.add_argument("-o", "--out-dir", type=Path, default=None, help="覆盖配置中的 file_directory")
    parser.add_argument("--enable-check", action="store_true",
                        help="开启数据质量检查（默认关闭，与 C++ 一致）；超差则不输出标定文件")
    parser.add_argument("--scan", action="store_true", help="只扫描静止窗口候选，不执行标定")
    parser.add_argument("--scan-topk", type=int, default=10, help="--scan 模式下列出的候选个数")
    parser.add_argument("--scan-step", type=float, default=1.0, help="--scan 模式下滑窗步长（秒）")
    args = parser.parse_args()

    cfg = load_config(args.config)
    params = build_params(cfg, args.enable_check)

    imu_file = args.imu_file or cfg.get("imu_file")
    if not imu_file:
        raise SystemExit("未指定 IMU 数据文件（配置中的 imu_file 或 --imu-file）")
    imu_file = Path(imu_file).expanduser()

    data = read_imu_file(imu_file)
    print(f"读取 {imu_file}：{data.shape[0]} 帧，"
          f"时间 {data[0, 0]:.6f} ~ {data[-1, 0]:.6f}（{data[-1, 0] - data[0, 0]:.3f} s）")

    if args.scan:
        scan_static_windows(data, params, args.scan_topk, args.scan_step)
        return 0

    start = args.start if args.start is not None else float(cfg.get("start_timestamp", 0) or 0)
    end = args.end if args.end is not None else float(cfg.get("end_timestamp", 0) or 0)
    window = select_range(data, start, end)
    if window.shape[0] < 3:
        raise SystemExit(f"指定时间范围内数据不足（{window.shape[0]} 帧），请检查 start/end 设置")

    result = calib_process(window, params)
    print_report(result, params)

    if result.error_code != int(ImuCalibErrorCode.CALIB_NO_ERROR):
        print("标定未通过检查，不输出标定参数文件。")
        return 1

    out_dir = args.out_dir or cfg.get("file_directory") or "."
    written = save_results(result, Path(out_dir).expanduser())
    print("标定参数文件已生成：")
    for p in written:
        print(f"  {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
