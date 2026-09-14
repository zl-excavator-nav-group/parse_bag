#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 ROS2 bag 中的消息解析出来，按 topic 分别转存为以空格分隔的文本文件。

特性：
- 用关键字列表过滤 topic（默认 "gnss,imu,encoder,gps"，topic 名包含任一关键字才解析）。
- 每个匹配的 topic 生成一个 .txt 文件，文件名由 topic 名而来（'/' 换成 '_'）。
- 每行第一列是 header 时间戳（单位：秒，精确到纳秒；无 header 的消息退回接收时间），后面各列是数据，空格分隔。
- 默认不写表头行；需要时用 --header 打开。
- 每个 topic 额外生成 <topic>_timing.txt 用于时序分析：第一列 header 时间戳(秒)、
  第二列接收时间(秒)、第三列 接收时间-header 时间(毫秒)。

各消息类型的列（均不含第一列时间戳）：
- IMU（sensor_msgs/Imu）：acc_x/y/z, gyro_x/y/z；
  加速度统一换算到 m/s^2（livox 驱动的 acc 单位是 g，会自动 ×9.80665）。
- 点云（Livox CustomMsg / sensor_msgs/PointCloud2）：写元信息 txt（CustomMsg 为
  header 时间/timebase/point_num/lidar_id，第一列是 header 时间）。
- 点云 / 图像按帧 dump（受脚本顶部 DUMP_MEDIA 一键开关 + POINTCLOUD_TOPICS /
  IMAGE_TOPICS + DUMP_START_TIME/DUMP_END_TIME 控制）：点云额外存 .pcd，图像存
  .jpg/.png（DUMP_IMG_FORMAT 选格式），文件名用 header.stamp（秒.纳秒）。
- GPS 经纬度（Gpgga / GnssPosMsg）：lat, lon, alt。
- GinsPvaMsg：lat, lon, alt, roll, pitch, yaw, vel_e, vel_n, vel_u, vel_yaw；
  同时把经纬度/姿态转到 UTM 坐标系，额外生成 <name>_utm.txt（车身 yaw）、
  <name>_vel_utm.txt（航向 vel_yaw）两份 TUM 文件，以及 <name>.geojson 轨迹文件；
  并对 <name>_utm.txt 做起点对齐（首帧为原点、仅保留 yaw），生成 gnss_odom.txt。
- GnssHdgMsg：hdg_type, baseline, heading, pitch；与位置 topic（GnssPosMsg / Gpgga）
  按时间戳就近匹配后，额外生成 <pos名>_hdg_utm.txt（UTM 下的 TUM 轨迹）、
  <pos名>_hdg_odom.txt（起点对齐轨迹）与 <pos名>.geojson。
- GnssVelMsg：vel_e, vel_n, vel_u。
- Odometry（nav_msgs/msg/Odometry）：直接输出 TUM 轨迹（timestamp tx ty tz qx qy qz qw），
  位置/姿态取自 pose.pose；同时生成 <topic>_timing.txt（header 时间 / 接收时间 / 延迟）。
- 其它匹配到但未单独定制的类型：通用展平（递归展开标量字段、跳过数组）。

运行前必须先 source 好 ROS2 与当前工作空间的环境：
    source /opt/ros/humble/setup.bash
    source ~/ws_excavator_nav/install/setup.bash

用法示例：
    python3 parse_bag.py
    python3 parse_bag.py --bag <bag目录> --out <输出根目录>
    # 结果写入 <输出根目录>/<bag名>_parsed_res/，例如
    # <输出根目录>/rosbag2_2026_08_20-09_24_26_parsed_res/
    python3 parse_bag.py --keywords gnss,imu,encoder,gps
    python3 parse_bag.py --header          # 写表头行
    python3 parse_bag.py --list-topics
    # 点云/图像按帧 dump：在脚本顶部设 DUMP_MEDIA=True、POINTCLOUD_TOPICS/IMAGE_TOPICS、
    # DUMP_START_TIME/DUMP_END_TIME 后直接运行；也可用命令行覆盖：
    python3 parse_bag.py --dump-media --pointcloud-topics /livox/lidar_192_168_1_158 \
        --image-topics /cam0/image_raw/compressed --start-time 1787218155 --end-time 1787218160
"""

import argparse
import re
import sys
from pathlib import Path

try:
    from rosbag2_py import (SequentialReader, StorageOptions, ConverterOptions,
                            StorageFilter)
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    raise SystemExit(
        "无法导入 ROS2 Python 库，请先 source 环境：\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source ~/ws_excavator_nav/install/setup.bash\n"
        f"原始错误: {e}"
    )

# utils.py（pva -> UTM/TUM 换算用）依赖 utm / geojson / scipy 等；
# 未安装时置 None，脚本仍可解析其它消息，只是跳过 pva 的 TUM 生成。
try:
    import utils
except Exception:
    utils = None

# ======================== 配置（按需修改） ========================
# 如果是mcap，直接到包目录；如果是sqlite3，指定 session 目录（含 metadata.yaml）
# BAG_PATH = "/home/chenchen/Data/2t_ex_data/0909/sensor_records/sensor_2026-09-09_143549/bag/session/"
BAG_PATH = "/home/chenchen/Data/2t_ex_data/rosbag2_2026_09_14-19_08_26/"
OUTPUT_DIR = "/home/chenchen/code/parse_bag/output_0914"

# topic 名（小写）包含任一关键字才解析；留空表示全部解析
# 点云 topic 名含 "lidar"（如 /livox/lidar_192_168_1_158），故加入 "lidar"
# KEYWORDS = ["gnss", "imu", "encoder", "gps", "lidar_192_168_1_158"]
KEYWORDS = ["gnss", "imu", "encoder", "gps", "localization", "odom", 'lidar', 'cam']
# KEYWORDS = ["lidar"]
# KEYWORDS = ["/cam0/image_raw/compressed", "/cam1/image_raw/compressed", "/cam2/image_raw/compressed", "/cam3/image_raw/compressed"]


# 是否写表头行（列名 + 单位）；默认 False（不带表头），可用 --header 打开
WRITE_HEADER = False

# 是否额外生成 <topic>_timing.txt（时序分析）：第一列 header 时间戳(秒)、
# 第二列接收时间(秒)、第三列 接收时间-header 时间(毫秒)
WRITE_TIMING = True

# IMU 加速度单位 -> m/s^2 的换算系数，按 topic 名匹配关键字。
# livox 驱动的 IMU，linear_acceleration 单位是 g（源码注释见
# livox_ros_driver2/src/comm/lidar_imu_data_queue.h: "Unit:g"），
# 因此这里 ×9.80665 转成 m/s^2；其它驱动默认认为已经是 m/s^2。
ACC_UNIT_TO_MPS2 = {
    "livox": 9.80665,
}
DEFAULT_ACC_SCALE = 1.0

# ---- 点云 / 图像按帧 dump 配置 ----
# 一键开关：True 时按帧 dump 点云(.pcd)/图像(.jpg/.png)；False 时点云仍写元信息 txt、图像不导出
DUMP_MEDIA = False

# 要 dump 的点云 / 图像 topic 名（精确匹配；留空则该类不 dump）
POINTCLOUD_TOPICS = ["/livox/lidar_192_168_1_158"]   # 例: ["/livox/lidar_192_168_1_158"]
IMAGE_TOPICS = []        # 例: ["/cam0/image_raw/compressed"]

# dump 时间范围（秒，epoch，可含小数），按消息 header.stamp 过滤；None 表示不限
DUMP_START_TIME = 1787218244
DUMP_END_TIME = 1787218245

# 图像输出格式：jpg / png
DUMP_IMG_FORMAT = "jpg"
# ==================================================================


# ---- 通用展平（用于“其它”未定制类型的兜底） ----

_PRIMITIVES = {
    # ROS2 的 get_fields_and_field_types() 返回的是 IDL 类型名，
    # 与 .msg 写法不完全一致：bool -> boolean, byte -> octet。
    # 这里同时收录 .msg 写法与 IDL 写法，避免把标量误判为嵌套消息。
    "bool", "boolean", "byte", "octet", "char", "wchar",
    "float", "double",
    "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "string", "wstring",
}
_ARRAY_RE = re.compile(r"^(.*)\[(\d*)\]$")                     # T[N] 或 T[]
_SEQ_RE = re.compile(r"^sequence\s*<\s*(.*?)\s*,\s*(\d+)\s*>$")  # sequence<T, N>
_SEQ_UNBOUNDED_RE = re.compile(r"^sequence\s*<\s*(.*?)\s*>$")    # sequence<T>


def _emit_message(msg, prefix, columns, max_array_len):
    for name, type_str in msg.get_fields_and_field_types().items():
        value = getattr(msg, name)
        col = f"{prefix}.{name}" if prefix else name
        _emit(value, type_str, col, columns, max_array_len)


def _emit(value, type_str, col, columns, max_array_len):
    m = _SEQ_RE.match(type_str)
    if m:
        _emit_array(value, m.group(1), col, columns, max_array_len, fixed=True)
        return
    m = _SEQ_UNBOUNDED_RE.match(type_str)
    if m:
        _emit_array(value, m.group(1), col, columns, max_array_len, fixed=False)
        return
    m = _ARRAY_RE.match(type_str)
    if m:
        fixed = m.group(2) != ""
        _emit_array(value, m.group(1), col, columns, max_array_len, fixed=fixed)
        return
    if type_str in _PRIMITIVES:
        columns.append((col, value))
        return
    # 其余视为嵌套消息类型（如 std_msgs/msg/Header）；没有嵌套消息接口的按标量兜底，
    # 避免遇到未收录的类型别名（如 boolean/octet）时对 bool/int 等调用失败。
    if hasattr(value, "get_fields_and_field_types"):
        _emit_message(value, col, columns, max_array_len)
    else:
        columns.append((col, value))


def _emit_array(value, elem_type, col, columns, max_array_len, fixed):
    n = len(value)
    if not fixed and (not max_array_len or n > max_array_len):
        return  # 变长数组默认跳过
    for i, item in enumerate(value):
        _emit(item, elem_type, f"{col}[{i}]", columns, max_array_len)


def flatten_message(msg, max_array_len=0):
    """把一条消息展平为 [(列名, 值), ...]。"""
    columns = []
    _emit_message(msg, "", columns, max_array_len)
    return columns


# ---- 各消息类型的列定义 ----

def acc_scale_for(topic):
    tl = topic.lower()
    for key, scale in ACC_UNIT_TO_MPS2.items():
        if key in tl:
            return scale
    return DEFAULT_ACC_SCALE


def fmt_imu(msg, topic):
    """IMU：acc_x/y/z(m/s^2), gyro_x/y/z(rad/s)。"""
    scale = acc_scale_for(topic)
    a = msg.linear_acceleration
    w = msg.angular_velocity
    cols = ["acc_x(m/s^2)", "acc_y(m/s^2)", "acc_z(m/s^2)",
            "gyro_x(rad/s)", "gyro_y(rad/s)", "gyro_z(rad/s)"]
    vals = [a.x * scale, a.y * scale, a.z * scale, w.x, w.y, w.z]
    return cols, vals


def fmt_llh(msg, topic):
    """GPS 经纬度：lat(deg), lon(deg), alt(m)。"""
    return ["lat(deg)", "lon(deg)", "alt(m)"], [msg.lat, msg.lon, msg.alt]


def fmt_llh_att(msg, topic):
    """GPS 经纬度 + 姿态角 + 速度：lat, lon, alt, roll, pitch, yaw, vel_e/n/u, vel_yaw。"""
    cols = ["lat(deg)", "lon(deg)", "alt(m)",
            "roll(deg)", "pitch(deg)", "yaw(deg)",
            "vel_e(m/s)", "vel_n(m/s)", "vel_u(m/s)", "vel_yaw(deg)"]
    vals = [msg.lat, msg.lon, msg.alt,
            msg.roll, msg.pitch, msg.yaw,
            msg.vel_e, msg.vel_n, msg.vel_u, msg.vel_yaw]
    return cols, vals


def fmt_hdg(msg, topic):
    """GNSS 航向：hdg_type, baseline(m), heading(deg), pitch(deg)。"""
    cols = ["hdg_type", "baseline(m)", "heading(deg)", "pitch(deg)"]
    vals = [msg.hdg_type, msg.baseline, msg.heading, msg.pitch]
    return cols, vals


def fmt_vel(msg, topic):
    """GNSS 速度（ENU）：vel_e(m/s), vel_n(m/s), vel_u(m/s)。"""
    cols = ["vel_e(m/s)", "vel_n(m/s)", "vel_u(m/s)"]
    vals = [msg.vel_e, msg.vel_n, msg.vel_u]
    return cols, vals


def fmt_odometry(msg, topic):
    """Odometry：输出 TUM 轨迹（timestamp tx ty tz qx qy qz qw）。

    位置/姿态取自 msg.pose.pose（geometry_msgs/Pose），本身已是笛卡尔位姿，
    无需 UTM 换算；第一列时间戳由主流程用 header.stamp 补上，因此该 topic 的
    .txt 文件即为 TUM 格式轨迹。
    """
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    cols = ["tx(m)", "ty(m)", "tz(m)", "qx", "qy", "qz", "qw"]
    vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    return cols, vals


def fmt_header_stamp(sec, nanosec):
    """std_msgs/Header.stamp(sec, nanosec) -> '秒.纳秒' 字符串（9 位纳秒）。"""
    sec = int(sec) + int(nanosec) // 1_000_000_000
    nsec = int(nanosec) % 1_000_000_000
    return f"{sec}.{nsec:09d}"


def fmt_custom_msg(msg, topic):
    """Livox CustomMsg 点云：header 时间、timebase、point_num、lidar_id。

    只导出点云帧的元信息（不导出点云本体）。header 时间作为第一列（秒，含纳秒），
    即 header.stamp；timebase 为点云首点时间（纳秒，uint64，与 header.stamp 同源）。
    """
    stamp = msg.header.stamp
    cols = ["header_time(s)", "timebase(ns)", "point_num", "lidar_id"]
    vals = [fmt_header_stamp(stamp.sec, stamp.nanosec),
            int(msg.timebase), int(msg.point_num), int(msg.lidar_id)]
    return cols, vals


def fmt_generic(msg, topic):
    """兜底：递归展平所有标量字段（跳过数组与字符串）。"""
    cols = flatten_message(msg, 0)
    return [c for c, _ in cols], [v for _, v in cols]


# 消息类型 -> 列定义函数
FORMATTERS = {
    "sensor_msgs/msg/Imu": fmt_imu,
    "nmea_msgs/msg/Gpgga": fmt_llh,
    "gnss_comm/msg/GnssPosMsg": fmt_llh,
    "gnss_comm/msg/GinsPvaMsg": fmt_llh_att,
    "gnss_comm/msg/GnssHdgMsg": fmt_hdg,
    "gnss_comm/msg/GnssVelMsg": fmt_vel,
    "nav_msgs/msg/Odometry": fmt_odometry,
    "livox_ros_driver2/msg/CustomMsg": fmt_custom_msg,
}

# 这些类型的 formatter 返回的 vals[0] 已经是时间字符串（如 Livox 点云的 header 时间），
# 写文件时不再在前面额外加 header 时间戳。
SELF_TIMESTAMPED_TYPES = {
    "livox_ros_driver2/msg/CustomMsg",
}


# ---- pva -> UTM / TUM 转换（复用 utils.py 的换算函数） ----

def convert_pva_to_tum(llh_file, tum_file, roll_col=4, pitch_col=5, yaw_col=6, zero_rp=False):
    """把 pva 的 llh 文件（timestamp lat lon alt ...）转成 TUM 格式。

    TUM 格式: timestamp tx ty tz qx qy qz qw
    tx/ty/tz 为 UTM easting / northing / 高程（米），qx qy qz qw 为姿态四元数。

    参数：
      roll_col/pitch_col/yaw_col: roll/pitch/yaw 所在的列（0 起，默认 4/5/6）。
      zero_rp: True 时把 roll/pitch 置 0（用于航向角 vel_yaw，它只有 2D 航向）。
    注意：pva 文件里的角度单位是度，本函数在传入 utils 前会先转成弧度。
    返回写入的帧数。
    """
    if utils is None:
        print("[警告] 未安装 utils.py 依赖（utm/geojson），跳过 TUM 生成。"
              "可执行: python3 -m pip install --user utm geojson", file=sys.stderr)
        return 0

    from scipy.spatial.transform import Rotation

    utils.reset_utm_zone()  # 每次生成前重置，首行数据固定 UTM zone
    n = 0
    with open(llh_file, encoding="utf-8") as fin, \
            open(tum_file, "w", encoding="utf-8") as fout:
        for line in fin:
            parts = line.split()
            if len(parts) < 7:
                continue
            try:
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3])
                roll = float(parts[roll_col]) * utils.DEG_TO_RAD
                pitch = float(parts[pitch_col]) * utils.DEG_TO_RAD
                yaw = float(parts[yaw_col]) * utils.DEG_TO_RAD   # 度 -> 弧度
            except (ValueError, IndexError):
                continue  # 表头行 / 非数据行

            if zero_rp:
                roll = pitch = 0.0

            easting, northing, zone_number, _ = utils.latlon_to_utm(lat, lon)
            utm_yaw = utils.ENUYawToUTMYaw(lat, lon, yaw, zone_number)
            rot = utils.RPY2mat(roll, pitch, utm_yaw)
            quat = Rotation.from_matrix(rot).as_quat()  # [x, y, z, w]

            fout.write(f"{parts[0]} {easting:.6f} {northing:.6f} {alt:.6f} "
                       f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n")
            n += 1
    return n


def align_traj_to_start_time(tum_file, start_tp=0, out_name=None):
    """把 TUM 轨迹对齐到起点坐标系（首帧为原点，仅保留首帧 yaw）。

    参考 run_parse_mcap.py 的 align_traj_to_start_time 与 sync_traj.adj_traj(only_yaw=True)：
    - 读入 TUM 文件（timestamp x y z qx qy qz qw）；
    - 若 start_tp > 0，丢弃时间戳早于 start_tp 的帧；
    - 以首帧位姿为基准，把 roll/pitch 置 0、只保留 yaw 构成起点变换 T_start，
      再对每帧做 pos.T = inv(T_start) @ pos.T（平移归零，姿态保持相对变化）；
    - 输出到同目录：start_tp==0 -> gnss_odom.txt，否则 aligned_start_<原名>；
      传了 out_name 时以 out_name 为准（多条链路各写各的，避免重名覆盖）。

    返回写入的帧数。
    """
    import math

    import numpy as np
    from scipy.spatial.transform import Rotation

    tum_path = Path(tum_file)
    out_dir = tum_path.parent
    output_path = out_dir / (out_name or ("gnss_odom.txt" if start_tp == 0
                                          else f"aligned_start_{tum_path.name}"))

    frames = []  # (ts_str, ts_float, T)
    with open(tum_path, encoding="utf-8") as fin:
        for line in fin:
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                ts = float(parts[0])
                xyz = [float(parts[1]), float(parts[2]), float(parts[3])]
                quat = [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]
            except ValueError:
                continue
            rot = Rotation.from_quat(quat).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rot
            T[:3, 3] = xyz
            frames.append((parts[0], ts, T))

    while frames and start_tp > 0 and frames[0][1] < start_tp:
        frames.pop(0)

    if not frames:
        print(f"[警告] {tum_path.name} 无有效数据，跳过起点对齐。", file=sys.stderr)
        return 0

    # 起点变换：只保留 yaw（等价 utils.mat2RPY(T_start[:3, :3])[2]），roll/pitch 置 0
    T_start = frames[0][2].copy()
    yaw = math.atan2(T_start[1, 0], T_start[0, 0])
    T_start[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    T_start_inv = np.linalg.inv(T_start)

    n = 0
    with open(output_path, "w", encoding="utf-8") as fout:
        for ts_str, _, T in frames:
            T_adj = T_start_inv @ T
            q = Rotation.from_matrix(T_adj[:3, :3]).as_quat()
            fout.write(f"{ts_str} {T_adj[0, 3]:.6f} {T_adj[1, 3]:.6f} {T_adj[2, 3]:.6f} "
                       f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")
            n += 1
    return n


def generate_geojson(llh_file, ts_interval=0.1):
    """调用 utils.create_geojson 生成轨迹 GeoJSON，返回 feature 数量。

    参考 run_parse_mcap.py 466/478 行的 utils.create_geojson 用法：把 llh 文件
    （timestamp lat lon ...）按 ts_interval 秒做时间抽稀，导出为 Point
    FeatureCollection，每个点的 properties 为 {time, info=最后一列}。
    """
    if utils is None:
        print("[警告] 未安装 utils.py 依赖（utm/geojson），跳过 GeoJSON 生成。",
              file=sys.stderr)
        return 0
    utils.create_geojson(str(llh_file), ts_interval=ts_interval)
    geo_file = Path(str(llh_file).replace(".txt", ".geojson"))
    if not geo_file.exists():
        return 0
    try:
        import json
        with open(geo_file, encoding="utf-8") as f:
            fc = json.load(f)
        return len(fc.get("features", []))
    except Exception:
        return -1


# ---- 输出辅助 ----

def fmt_timestamp(ns):
    """纳秒 -> 秒字符串，精确到小数点后 9 位（避免浮点精度丢失）。"""
    sec, nsec = divmod(int(ns), 1_000_000_000)
    return f"{sec}.{nsec:09d}"


def fmt_value(v):
    """把单个值转成文本。bool -> 1/0，float -> 最短可往返表示。"""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        return repr(v)
    return str(v)


def sanitize_topic_name(topic):
    """'/gps/pos' -> 'gps_pos'。"""
    return topic.strip("/").replace("/", "_") or "root"


def detect_storage_id(bag_dir):
    bag_dir = Path(bag_dir)
    meta = bag_dir / "metadata.yaml"
    if meta.exists():
        # 有的 bag 的 storage_identifier 是空字符串 ""，这里容错：剥掉引号后若为空则走兜底
        m = re.search(r"storage_identifier:\s*(.*)", meta.read_text(errors="ignore"))
        if m:
            sid = m.group(1).strip().strip("'\"")
            if sid:
                return sid
    # 兜底：按 bag 目录内的文件后缀判断（.mcap -> mcap，否则 sqlite3）
    if any(bag_dir.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


# ---- 二进制 dump（点云 -> .pcd，图像 -> .png/.jpg） ----

# 按帧落盘成独立文件的类型：这些 topic 不写文本，而是每帧一个文件。
POINTCLOUD_TYPES = {
    "livox_ros_driver2/msg/CustomMsg",
    "sensor_msgs/msg/PointCloud2",
}
IMAGE_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
}

# PointField.datatype -> numpy 小端 dtype
_PCD_FIELD_DTYPES = {
    1: "<i1", 2: "<u1", 3: "<i2", 4: "<u2",
    5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8",
}


def parse_time_arg(s):
    """把 '秒' / '秒.纳秒' 字符串转成整纳秒；None 或空表示不限。"""
    if s is None:
        return None
    s = str(s).strip()
    if not s:
        return None
    return int(round(float(s) * 1_000_000_000))


def header_stamp_ns(msg):
    """消息 header.stamp -> 整纳秒。"""
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def message_header_ns(msg):
    """消息 header.stamp -> 整纳秒；消息无 header 时返回 None。"""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def custom_msg_to_pcd(msg):
    """Livox CustomMsg -> PCD 二进制字节（x y z intensity，intensity=reflectivity）。"""
    import numpy as np

    points = msg.points
    n = len(points)
    arr = np.zeros((n, 4), dtype="<f4")
    for i, p in enumerate(points):
        arr[i] = (p.x, p.y, p.z, p.reflectivity)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    return header.encode("ascii") + arr.tobytes()


def pointcloud2_to_pcd(msg):
    """sensor_msgs/PointCloud2 -> PCD 二进制字节（x y z intensity，缺 intensity 则置 0）。"""
    import numpy as np

    step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if step <= 0 or n <= 0:
        n = 0
    elif len(buf) < step * n:
        n = len(buf) // step
    mat = buf.reshape(n, step) if n else np.zeros((0, step), dtype=np.uint8)

    def extract(name):
        for f in msg.fields:
            if f.name != name or f.count < 1:
                continue
            dt = np.dtype(_PCD_FIELD_DTYPES.get(f.datatype))
            if dt is None:
                return None
            size = dt.itemsize * f.count
            col = mat[:, f.offset:f.offset + size]
            return np.ascontiguousarray(col).view(dt).reshape(n)[:].astype("<f4")
        return None

    names = ["x", "y", "z"]
    cols = [extract("x"), extract("y"), extract("z")]
    if extract("intensity") is not None:
        names.append("intensity")
        cols.append(extract("intensity"))
    cols = [c if c is not None else np.zeros(n, dtype="<f4") for c in cols]
    arr = np.stack(cols, axis=1) if n else np.zeros((0, len(names)), dtype="<f4")

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS " + " ".join(names) + "\n"
        + "SIZE " + " ".join(["4"] * len(names)) + "\n"
        + "TYPE " + " ".join(["F"] * len(names)) + "\n"
        + "COUNT " + " ".join(["1"] * len(names)) + "\n"
        + f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        + f"POINTS {n}\n"
        "DATA binary\n"
    )
    return header.encode("ascii") + arr.tobytes()


def compressed_image_to_bytes(msg, prefer):
    """CompressedImage -> (bytes, 扩展名)。源格式与目标一致时直接复制，否则解码重编码。"""
    import cv2
    import numpy as np

    fmt = (getattr(msg, "format", "") or "").lower()
    raw = bytes(msg.data)
    if prefer == "jpg" and fmt in ("jpeg", "jpg"):
        return raw, ".jpg"
    if prefer == "png" and fmt == "png":
        return raw, ".png"

    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"无法解码压缩图像 (format={fmt})")
    ext = ".jpg" if prefer == "jpg" else ".png"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        raise ValueError(f"编码为 {ext} 失败")
    return buf.tobytes(), ext


def raw_image_to_bytes(msg, prefer):
    """sensor_msgs/Image -> (bytes, 扩展名)。"""
    import cv2
    from cv_bridge import CvBridge

    img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    ext = ".jpg" if prefer == "jpg" else ".png"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        raise ValueError(f"编码为 {ext} 失败")
    return buf.tobytes(), ext


def dump_media_message(msg, ttype, topic, out_dir, start_ns, end_ns, img_format,
                       media_counts, warn_once):
    """把一帧点云/图像落盘到 <out_dir>/<topic名>/<header.stamp>.<ext>；返回是否写出。"""
    ts_ns = header_stamp_ns(msg)
    if start_ns is not None and ts_ns < start_ns:
        return False
    if end_ns is not None and ts_ns > end_ns:
        return False

    sub = out_dir / sanitize_topic_name(topic)
    sub.mkdir(parents=True, exist_ok=True)
    stamp = msg.header.stamp
    name = fmt_header_stamp(stamp.sec, stamp.nanosec)

    try:
        if ttype in POINTCLOUD_TYPES:
            payload = (pointcloud2_to_pcd(msg) if ttype == "sensor_msgs/msg/PointCloud2"
                       else custom_msg_to_pcd(msg))
            ext = ".pcd"
        elif ttype == "sensor_msgs/msg/CompressedImage":
            payload, ext = compressed_image_to_bytes(msg, img_format)
        else:  # sensor_msgs/msg/Image
            payload, ext = raw_image_to_bytes(msg, img_format)
    except Exception as e:
        if topic not in warn_once:
            warn_once.add(topic)
            print(f"[警告] {topic}: 帧 {name} dump 失败: {e}", file=sys.stderr)
        return False

    # 防重名：同一时间戳出现多帧时加 _1/_2/... 后缀
    path = sub / (name + ext)
    k = 1
    while path.exists():
        path = sub / f"{name}_{k}{ext}"
        k += 1
    path.write_bytes(payload)
    media_counts[topic] = media_counts.get(topic, 0) + 1
    return True


# ---- 主流程 ----

def main():
    ap = argparse.ArgumentParser(description="ROS2 bag -> 文本文件（每个 topic 一个 txt）")
    ap.add_argument("--bag", default=BAG_PATH, help="rosbag 目录路径")
    ap.add_argument("--out", default=OUTPUT_DIR,
                    help="输出根目录，实际结果写入 <out>/<bag名>_parsed_res")
    ap.add_argument("--keywords", default=",".join(KEYWORDS),
                    help="逗号分隔的关键字，topic 名包含任一关键字才解析；留空则全部解析")
    ap.add_argument("--topics", action="append", default=[],
                    help="精确指定 topic 名（逗号分隔，可重复）；传入后忽略关键字过滤")
    ap.add_argument("--storage", default=None, help="存储格式 sqlite3/mcap，默认自动识别")
    ap.add_argument("--sep", default=" ", help="列分隔符（默认单个空格）")
    ap.add_argument("--header", action="store_true", help="写表头行（默认不写）")
    ap.add_argument("--timing", dest="write_timing", action="store_true", default=None,
                    help="额外生成 <topic>_timing.txt（默认按脚本顶部 WRITE_TIMING）")
    ap.add_argument("--no-timing", dest="write_timing", action="store_false", default=None,
                    help="不生成 <topic>_timing.txt")
    ap.add_argument("--list-topics", action="store_true", help="仅列出 topic 与类型后退出")
    ap.add_argument("--dump-media", dest="dump_media", action="store_true", default=None,
                    help="开启点云/图像 dump（覆盖脚本顶部 DUMP_MEDIA）")
    ap.add_argument("--no-dump-media", dest="dump_media", action="store_false", default=None,
                    help="关闭点云/图像 dump（覆盖脚本顶部 DUMP_MEDIA）")
    ap.add_argument("--pointcloud-topics", default=None,
                    help="逗号分隔的点云 topic 名，覆盖脚本顶部 POINTCLOUD_TOPICS")
    ap.add_argument("--image-topics", default=None,
                    help="逗号分隔的图像 topic 名，覆盖脚本顶部 IMAGE_TOPICS")
    ap.add_argument("--start-time", default=None,
                    help="时间范围起点（秒，epoch，可含小数/纳秒），只 dump 此后的点云/图像帧；默认取脚本顶部 DUMP_START_TIME")
    ap.add_argument("--end-time", default=None,
                    help="时间范围终点（秒，epoch），只 dump 此前的点云/图像帧；默认取脚本顶部 DUMP_END_TIME")
    ap.add_argument("--img-format", default=None, choices=["jpg", "png"],
                    help="图像输出格式 jpg/png（默认取脚本顶部 DUMP_IMG_FORMAT；源已同格式时直接复制不重编码）")
    args = ap.parse_args()

    write_header = WRITE_HEADER or args.header
    write_timing = WRITE_TIMING if args.write_timing is None else args.write_timing
    dump_media = DUMP_MEDIA if args.dump_media is None else args.dump_media
    img_format = (args.img_format or DUMP_IMG_FORMAT).lower()
    start_arg = args.start_time if args.start_time is not None else DUMP_START_TIME
    end_arg = args.end_time if args.end_time is not None else DUMP_END_TIME
    start_ns = parse_time_arg(start_arg)
    end_ns = parse_time_arg(end_arg)

    pointcloud_topics = ([t.strip() for t in args.pointcloud_topics.split(",") if t.strip()]
                         if args.pointcloud_topics is not None else list(POINTCLOUD_TOPICS))
    image_topics = ([t.strip() for t in args.image_topics.split(",") if t.strip()]
                    if args.image_topics is not None else list(IMAGE_TOPICS))
    dump_topics = set(pointcloud_topics) | set(image_topics)

    bag_dir = str(Path(args.bag))
    storage_id = args.storage or detect_storage_id(bag_dir)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )

    all_topics = reader.get_all_topics_and_types()
    if args.list_topics:
        for tm in sorted(all_topics, key=lambda t: t.name):
            print(f"{tm.name}\t{tm.type}")
        return

    # 选择要解析的 topic：精确名 > 关键字
    exact = set()
    for group in args.topics:
        for t in group.split(","):
            t = t.strip()
            if t:
                exact.add(t)
    keywords = [k.strip().lower() for k in args.keywords.split(",") if k.strip()]

    if exact:
        matched = [tm for tm in all_topics if tm.name in exact]
    elif keywords:
        matched = [tm for tm in all_topics if any(k in tm.name.lower() for k in keywords)]
    else:
        matched = list(all_topics)

    # 把配置/命令行指定的点云、图像 dump topic 合并进匹配集（不干扰上面的关键字/--topics 选择）
    if dump_topics:
        existing = {tm.name for tm in matched}
        matched.extend(tm for tm in all_topics
                       if tm.name in dump_topics and tm.name not in existing)

    if not matched:
        print("没有匹配的 topic。可用 --list-topics 查看 bag 里的 topic。", file=sys.stderr)
        return

    print(f"关键字: {keywords if keywords else '(无，全量)'}")
    print(f"匹配到 {len(matched)} 个 topic:")
    for tm in sorted(matched, key=lambda t: t.name):
        if tm.name in dump_topics and (tm.type in POINTCLOUD_TYPES or tm.type in IMAGE_TYPES):
            if dump_media:
                desc = ("pcd 帧 dump" if tm.type in POINTCLOUD_TYPES
                        else f"图像帧 dump ({img_format})")
            elif tm.type in POINTCLOUD_TYPES:
                desc = "点云元信息 txt（dump 已关闭）"
            else:
                desc = "图像（dump 已关闭，跳过）"
        else:
            desc = FORMATTERS.get(tm.type, fmt_generic).__name__
        print(f"  {tm.name:40s} {tm.type:32s} -> {desc}")

    reader.set_filter(StorageFilter(topics=sorted(tm.name for tm in matched)))

    # topic 名 -> 类型字符串；并解析消息类（缺包则跳过）
    topic_type = {tm.name: tm.type for tm in matched}
    msg_class = {}
    skipped = []
    for tm in matched:
        try:
            msg_class[tm.name] = get_message(tm.type)
        except Exception as e:
            skipped.append((tm.name, tm.type, str(e)))
    for name, typ, err in skipped:
        print(f"[跳过] {name} ({typ}): 无法解析消息类型: {err}", file=sys.stderr)

    # 输出到 <out>/<bag目录名>_parsed_res，如 rosbag2_2026_08_20-09_24_26_parsed_res
    out_dir = Path(args.out) / (Path(bag_dir).name + "_parsed_res")
    out_dir.mkdir(parents=True, exist_ok=True)

    handles = {}
    timing_handles = {}
    counts = {}
    media_counts = {}
    media_warn_once = set()
    generic_warn = set()
    total = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        cls = msg_class.get(topic)
        if cls is None:
            continue

        msg = deserialize_message(data, cls)
        ttype = topic_type[topic]
        h_ns = message_header_ns(msg)  # 该消息的 header 时间戳（纳秒）；无 header 为 None

        # 时序分析：额外写 <topic>_timing.txt（header 时间 / 接收时间 / 延迟）
        if write_timing:
            if h_ns is not None:
                if topic not in timing_handles:
                    tpath = out_dir / (sanitize_topic_name(topic) + "_timing.txt")
                    timing_handles[topic] = open(tpath, "w", encoding="utf-8")
                    if write_header:
                        timing_handles[topic].write(
                            args.sep.join(["header_time(s)", "recv_time(s)", "delay(ms)"]) + "\n")
                delay_ms = (int(t) - h_ns) / 1_000_000.0
                timing_handles[topic].write(
                    args.sep.join([fmt_timestamp(h_ns), fmt_timestamp(t),
                                   f"{delay_ms:.6f}"]) + "\n")

        # 二进制 dump：仅当开启且该 topic 在配置列表里（点云 -> .pcd，图像 -> .jpg/.png）
        if (dump_media and topic in dump_topics
                and (ttype in POINTCLOUD_TYPES or ttype in IMAGE_TYPES)):
            dump_media_message(msg, ttype, topic, out_dir, start_ns, end_ns,
                               img_format, media_counts, media_warn_once)

        # 文本：图像不写文本；点云走 fmt_custom_msg 写元信息 txt；其它类型照旧
        if ttype in IMAGE_TYPES:
            continue

        formatter = FORMATTERS.get(ttype, fmt_generic)
        cols, vals = formatter(msg, topic)

        # 自身携带时间戳的类型（如 Livox 点云的 header 时间）不再在前面额外加时间戳
        self_ts = ttype in SELF_TIMESTAMPED_TYPES

        if topic not in handles:
            path = out_dir / (sanitize_topic_name(topic) + ".txt")
            handles[topic] = open(path, "w", encoding="utf-8")
            counts[topic] = 0
            if ttype not in FORMATTERS:
                generic_warn.add(topic)
            if write_header:
                header_cols = cols if self_ts else ["header_time(s)"] + cols
                handles[topic].write(args.sep.join(header_cols) + "\n")

        if self_ts:
            row = [fmt_value(v) for v in vals]
        else:
            # 第一列统一用 header 时间（算法消费的时间基准）；无 header 时退回接收时间
            ts_ns = h_ns if h_ns is not None else int(t)
            row = [fmt_timestamp(ts_ns)] + [fmt_value(v) for v in vals]
        handles[topic].write(args.sep.join(row) + "\n")
        counts[topic] += 1
        total += 1

        if total % 100000 == 0:
            print(f"... 已处理 {total} 条", file=sys.stderr)

    for f in handles.values():
        f.close()

    # pva 后处理：llh -> UTM + TUM + GeoJSON（参考 run_parse_mcap.py 461~468 行的做法）
    for tm in matched:
        if tm.type == "gnss_comm/msg/GinsPvaMsg" and counts.get(tm.name, 0) > 0:
            base = sanitize_topic_name(tm.name)
            llh_path = out_dir / (base + ".txt")
            body_path = out_dir / (base + "_utm.txt")      # 车身姿态（roll/pitch/yaw）
            vel_path = out_dir / (base + "_vel_utm.txt")   # 航向（yaw=vel_yaw，roll/pitch 置 0）
            n1 = convert_pva_to_tum(llh_path, body_path, roll_col=4, pitch_col=5, yaw_col=6)
            n2 = convert_pva_to_tum(llh_path, vel_path, roll_col=4, pitch_col=5, yaw_col=10,
                                    zero_rp=True)
            n3 = generate_geojson(llh_path, ts_interval=0.1)
            n4 = align_traj_to_start_time(body_path, start_tp=0)
            print(f"  生成 TUM(车身 yaw):     {body_path.name} ({n1} 帧)")
            print(f"  生成 TUM(航向 vel_yaw): {vel_path.name} ({n2} 帧)")
            print(f"  生成 GeoJSON(轨迹):     {base}.geojson ({n3} 帧)")
            print(f"  生成起点对齐轨迹:      gnss_odom.txt ({n4} 帧)")

    # GNSS pos + hdg 后处理：位置(llh) 与 航向(yaw/pitch) 按时间戳匹配 -> UTM TUM
    pos_topics = [tm for tm in matched
                  if tm.type == "gnss_comm/msg/GnssPosMsg" and counts.get(tm.name, 0) > 0]
    if not pos_topics:
        pos_topics = [tm for tm in matched
                      if tm.type == "nmea_msgs/msg/Gpgga" and counts.get(tm.name, 0) > 0]
    hdg_topics = [tm for tm in matched
                  if tm.type == "gnss_comm/msg/GnssHdgMsg" and counts.get(tm.name, 0) > 0]
    for pos_tm, hdg_tm in zip(pos_topics, hdg_topics):
        base = sanitize_topic_name(pos_tm.name)
        pos_path = out_dir / (base + ".txt")
        hdg_path = out_dir / (sanitize_topic_name(hdg_tm.name) + ".txt")
        tum_path = out_dir / (base + "_hdg_utm.txt")
        if utils is not None:
            n = utils.combine_pos_hdg_to_tum(pos_path, hdg_path, tum_path)
            print(f"  生成 TUM(pos+hdg):      {tum_path.name} ({n} 帧)")
            if n > 0:
                # 起点对齐用 <pos名>_hdg_odom.txt，避免与 pva 的 gnss_odom.txt 重名；
                # GeoJSON 取经纬度原文件（TUM 是 UTM 坐标，create_geojson 只吃 lat/lon）
                odom_name = base + "_hdg_odom.txt"
                n_odom = align_traj_to_start_time(tum_path, start_tp=0,
                                                  out_name=odom_name)
                n_geo = generate_geojson(pos_path, ts_interval=0.1)
                print(f"  生成起点对齐轨迹:      {odom_name} ({n_odom} 帧)")
                print(f"  生成 GeoJSON(轨迹):     {base}.geojson ({n_geo} 帧)")

    # 汇总
    print(f"\n共导出 {total} 条消息，输出目录: {out_dir.resolve()}")
    for tm in sorted(matched, key=lambda t: t.name):
        ttype = tm.type
        name = tm.name

        # 图像：只做二进制 dump，不写文本
        if ttype in IMAGE_TYPES:
            if dump_media and name in dump_topics:
                n = media_counts.get(name, 0)
                sub = sanitize_topic_name(name)
                if n == 0:
                    print(f"  {name:40s} 0 帧（该 topic 在时间范围内无数据）")
                else:
                    print(f"  {name:40s} {n:>8} 帧 -> {sub}/ (每帧 .{img_format})")
            else:
                print(f"  {name:40s} 未 dump（DUMP_MEDIA 关闭或未在 IMAGE_TOPICS 中）")
            continue

        # 文本（点云在此写元信息 txt）
        n = counts.get(name, 0)
        path = out_dir / (sanitize_topic_name(name) + ".txt")
        note = ""
        if name in generic_warn:
            note = "  [通用格式，如需定制请告诉我]"
        if n == 0:
            print(f"  {name:40s} 0 条（该 topic 无数据，未生成文件）")
        else:
            print(f"  {name:40s} {n:>8} 条 -> {path.name}{note}")

        # 点云额外 dump 的 PCD
        if ttype in POINTCLOUD_TYPES and dump_media and name in dump_topics:
            m = media_counts.get(name, 0)
            sub = sanitize_topic_name(name)
            print(f"  {'':40s} {m:>8} 帧 -> {sub}/ (每帧 .pcd)")


if __name__ == "__main__":
    main()
