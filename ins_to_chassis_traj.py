#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""由组合惯导轨迹 + 回转编码器角度，推算底盘（履带）在世界系下的位姿轨迹。

挖掘机上车与履带底盘之间可绕回转中心相对转动，组合惯导装在上车上，所以
parse_bag.py 导出的 gps_pva_utm.txt 描述的是【上车】的位姿，不是底盘的位姿。
本脚本按下面的链路把上车轨迹换算成底盘轨迹：

    T_world_chassis = T_world_ins @ T_ins_base @ T_base_chassis(theta)

其中：
    T_world_ins     组合惯导在世界（UTM）系下的位姿，来自 TUM 轨迹文件；
    T_ins_base      回转中心在组合惯导系下的固定外参，
                    默认平移 (0.65, -0.052, 0) m，rpy (0, 0, 1) 度；
    T_base_chassis  纯 Rz(theta)，无平移，theta 直接取回转编码器角度（度）。

注意 T_base_chassis 的平移为 0，所以底盘原点在世界系下的位置
    t_world_chassis = t_world_ins + R_world_ins @ t_ins_base
【与编码器角度完全无关】——它就是回转中心的世界轨迹，编码器只影响姿态。

输入：
    组合惯导轨迹（TUM，8 列）: timestamp easting northing alt qx qy qz qw
    回转编码器状态（22 列无表头），角度取【0 起索引的第 9 列】encoder_position_deg，
    单位是度。注意 0 起索引的第 8 列是 counts_per_revolution = 16384，别取错。

输出：
    同样的 TUM 8 列格式，时间戳沿用惯导时间戳（编码器角度线性插值到惯导时刻）。

用法示例：
    python3 ins_to_chassis_traj.py
    python3 ins_to_chassis_traj.py --dir parse_res_0826/rosbag2_2026_08_26-03_56_05_parsed_res
    python3 ins_to_chassis_traj.py --dir <目录> --fill-head        # 保留起始缺口帧
    python3 ins_to_chassis_traj.py --dir <目录> --angle-sign -1    # 反转编码器正方向
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# tools/ 下没有 __init__.py，不是包，直接把它加进 sys.path 再导入。
# （my_test_tools 会把 tools/ 加进 sys.path 来 import utils，导入是安全的。）
sys.path.append(str(Path(__file__).resolve().parent / "tools"))
from my_test_tools import xyzrpy_deg_to_se3  # noqa: E402

# ======================== 配置（默认值，命令行可覆盖） ========================
# 默认数据目录（不传 --dir / --ins 时用它）
DEFAULT_DIR = "/home/chenchen/code/parse_bag/output_0827/rosbag2_2026_08_27-08_42_04_parsed_res"

INS_NAME = "gps_pva_utm.txt"
ENCODER_NAME = "excavator_swing_encoder_state.txt"
OUT_NAME = "chassis_utm.txt"

# T_ins_base：回转中心（base）在组合惯导（ins）系下的位姿
EXT_XYZ = (0.65, -0.052, 0.0)       # 平移，米
EXT_RPY_DEG = (0.0, 0.0, 1.0)       # roll / pitch / yaw，度（ZYX）

# 回转编码器角度所在列（0 起索引）：9 -> encoder_position_deg（度）
ENCODER_ANGLE_COL = 9

MAX_ENCODER_GAP = 1.0               # 编码器插值允许的最大空洞（秒）
VALID_WIN = 25                      # 航向校验滑动窗口（帧）
VALID_MIN_DIST = 0.3                # 航向校验最小位移（米）
# ============================================================================


def wrap180(angle_deg):
    """把角度（度）回绕到 [-180, 180)。"""
    return (np.asarray(angle_deg) + 180.0) % 360.0 - 180.0


def read_tum_with_raw_ts(path):
    """读 TUM 轨迹文件，返回 (ts, ts_str, xyz, quat)。

    ts_str 保留时间戳的【原始字符串】：float64 在 1.78e9 这个量级只有约 0.4us
    分辨率，float("1787716566.365107465") 再用 %.9f 打印会变成 ...107536，
    导致输出时间戳与输入对不上。所以输出时必须原样写回字符串。

    quat 为 xyzw 顺序（与 parse_bag.py 写出的一致）。
    """
    ts, ts_str, xyz, quat = [], [], [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                vals = [float(v) for v in parts[:8]]
            except ValueError:
                continue                        # 表头行 / 非数据行
            ts.append(vals[0])
            ts_str.append(parts[0])
            xyz.append(vals[1:4])
            quat.append(vals[4:8])
    return (np.asarray(ts, dtype=np.float64), ts_str,
            np.asarray(xyz, dtype=np.float64), np.asarray(quat, dtype=np.float64))


def read_encoder(path, col, sign=1.0, offset_deg=0.0):
    """读回转编码器角度，返回按时间升序的 (ts, angle_deg)。

    angle = sign * 原始角度 + offset_deg（都在“度”的量纲下施加）。
    """
    data = np.loadtxt(path, usecols=(0, col), dtype=np.float64)
    ts = data[:, 0]
    ang = sign * data[:, 1] + offset_deg
    order = np.argsort(ts, kind="stable")       # np.interp 要求 xp 单调递增
    return ts[order], ang[order]


def interp_encoder_deg(ins_ts, enc_ts, enc_deg, max_gap, fill_head=False):
    """把编码器角度插值到惯导时刻，返回 (theta_deg, valid, stats)。

    关键：必须先【解缠】再插值。若直接对回绕过的角度做线性插值，一次
    360->0 的跳变会插出一整圈假回转（参考 tools/compare_yaw_swing_encoder.py
    里的 unwrap_deg）。

    valid 标记哪些惯导时刻有可信的编码器角度：
      - 早于编码器起始时刻的帧：fill_head=False 时丢弃，True 时用首值填充；
      - 晚于编码器结束时刻的帧：一律丢弃；
      - 落在超过 max_gap 的编码器空洞里的帧：丢弃。
    """
    enc_rad = np.unwrap(np.radians(enc_deg))
    theta = np.degrees(np.interp(ins_ts, enc_ts, enc_rad))   # 越界自动取端值

    n_head = int(np.sum(ins_ts < enc_ts[0]))
    n_tail = int(np.sum(ins_ts > enc_ts[-1]))

    valid = ins_ts <= enc_ts[-1]
    if not fill_head:
        valid &= ins_ts >= enc_ts[0]

    n_gap = 0
    if max_gap and max_gap > 0:
        hi = np.clip(np.searchsorted(enc_ts, ins_ts, side="left"), 0, len(enc_ts) - 1)
        lo = np.clip(hi - 1, 0, len(enc_ts) - 1)
        # 注意不能用 np.isclose：它在 1.79e9 的秒级纪元下默认 rtol 会把
        # 数十秒都当成“相等”，导致 on_node 恒为 True、空洞永远检测不到。
        on_node = np.abs(ins_ts - enc_ts[hi]) < 1e-9          # 恰好落在采样点上
        gap = np.where(on_node, 0.0, enc_ts[hi] - enc_ts[lo])
        gap_bad = (gap > max_gap) & valid
        n_gap = int(gap_bad.sum())
        valid &= ~gap_bad

    return theta, valid, {"head": n_head, "tail": n_tail, "gap": n_gap}


def build_chassis_poses(xyz, quat, theta_deg, T_ins_base):
    """按 T_world_chassis = T_world_ins @ T_ins_base @ Rz(theta) 批量组合位姿。

    返回 (xyz_chassis, quat_chassis(xyzw), yaw_chassis_deg)。
    """
    R_wi = Rotation.from_quat(quat).as_matrix()      # (N,3,3)，输入 xyzw
    R_ib = np.asarray(T_ins_base[:3, :3], dtype=np.float64)
    t_ib = np.asarray(T_ins_base[:3, 3], dtype=np.float64)

    # T_base_chassis 平移为 0，所以底盘原点位置与编码器角度无关
    xyz_ch = xyz + R_wi @ t_ib                       # (N,3,3)@(3,) -> (N,3)

    th = np.radians(theta_deg)
    c, s = np.cos(th), np.sin(th)
    Rz = np.zeros((len(th), 3, 3), dtype=np.float64)
    Rz[:, 0, 0] = c
    Rz[:, 0, 1] = -s
    Rz[:, 1, 0] = s
    Rz[:, 1, 1] = c
    Rz[:, 2, 2] = 1.0

    R_wc = R_wi @ R_ib @ Rz                          # (N,3,3)
    quat_ch = Rotation.from_matrix(R_wc).as_quat()   # xyzw
    yaw_ch = np.degrees(np.arctan2(R_wc[:, 1, 0], R_wc[:, 0, 0]))
    return xyz_ch, quat_ch, yaw_ch


def write_tum(path, ts_str, xyz, quat):
    """写 TUM 文件：timestamp x y z qx qy qz qw（与 parse_bag.py 的写法一致）。

    时间戳原样透传字符串，位姿保留 6 位小数。
    """
    with open(path, "w", encoding="utf-8") as f:
        for s, p, q in zip(ts_str, xyz, quat):
            f.write(f"{s} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                    f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")


def heading_vs_travel(xyz, yaw_deg, win, min_dist):
    """用“履带只能沿底盘 x 轴走”这一约束校验底盘航向。

    对底盘原点 XY 取滑动窗位移向量求方位角，只统计位移大于 min_dist 的样本，
    分别报告与 yaw 和 yaw+180 的中位绝对误差。返回 None 表示样本不足。
    """
    if len(xyz) <= win:
        return None
    d = xyz[win:, :2] - xyz[:-win, :2]
    dist = np.linalg.norm(d, axis=1)
    mask = dist > min_dist
    if not mask.any():
        return None
    bearing = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
    err = wrap180(bearing - yaw_deg[:len(bearing)])
    fwd = float(np.median(np.abs(err[mask])))
    rev = float(np.median(np.abs(wrap180(err[mask] - 180.0))))
    return fwd, rev, int(mask.sum())


def rate_of(ts):
    """估算采样率（Hz）。"""
    span = ts[-1] - ts[0]
    return (len(ts) - 1) / span if span > 0 else float("nan")


def main():
    ap = argparse.ArgumentParser(
        description="组合惯导轨迹 + 回转编码器 -> 底盘世界系轨迹")
    ap.add_argument("--dir", default=DEFAULT_DIR,
                    help=f"数据目录，自动拼 {INS_NAME} / {ENCODER_NAME} / {OUT_NAME}")
    ap.add_argument("--ins", default=None, help="组合惯导 TUM 轨迹文件（覆盖 --dir）")
    ap.add_argument("--encoder", default=None, help="回转编码器状态文件（覆盖 --dir）")
    ap.add_argument("--out", default=None, help="输出 TUM 文件（覆盖 --dir）")

    ap.add_argument("--angle-col", type=int, default=ENCODER_ANGLE_COL,
                    help="编码器角度所在列（0 起索引，默认 9 = encoder_position_deg）")
    ap.add_argument("--angle-sign", type=float, default=1.0,
                    help="编码器角度符号，默认 +1")
    ap.add_argument("--angle-offset-deg", type=float, default=0.0,
                    help="编码器角度常值偏置（度），默认 0")

    ap.add_argument("--ext-x", type=float, default=EXT_XYZ[0])
    ap.add_argument("--ext-y", type=float, default=EXT_XYZ[1])
    ap.add_argument("--ext-z", type=float, default=EXT_XYZ[2])
    ap.add_argument("--ext-roll", type=float, default=EXT_RPY_DEG[0], help="外参 roll（度）")
    ap.add_argument("--ext-pitch", type=float, default=EXT_RPY_DEG[1], help="外参 pitch（度）")
    ap.add_argument("--ext-yaw", type=float, default=EXT_RPY_DEG[2], help="外参 yaw（度）")

    ap.add_argument("--max-gap", type=float, default=MAX_ENCODER_GAP,
                    help="编码器插值允许的最大空洞（秒），默认 1.0")
    ap.add_argument("--fill-head", action="store_true",
                    help="用编码器首值填充起始缺口（默认丢弃这些帧）")
    ap.add_argument("--valid-win", type=int, default=VALID_WIN,
                    help="航向校验滑动窗口（帧）")
    ap.add_argument("--valid-min-dist", type=float, default=VALID_MIN_DIST,
                    help="航向校验最小位移（米）")
    args = ap.parse_args()

    base = Path(args.dir)
    ins_path = Path(args.ins) if args.ins else base / INS_NAME
    enc_path = Path(args.encoder) if args.encoder else base / ENCODER_NAME
    out_path = Path(args.out) if args.out else base / OUT_NAME

    for p in (ins_path, enc_path):
        if not p.exists():
            ap.error(f"文件不存在: {p}")

    # ---- 1. 读输入 ----
    ins_ts, ins_ts_str, ins_xyz, ins_quat = read_tum_with_raw_ts(ins_path)
    if len(ins_ts) == 0:
        ap.error(f"没有从 {ins_path} 读到任何位姿")
    enc_ts, enc_deg = read_encoder(enc_path, args.angle_col,
                                   args.angle_sign, args.angle_offset_deg)
    if len(enc_ts) == 0:
        ap.error(f"没有从 {enc_path} 读到任何编码器角度")

    # ---- 2. 外参 T_ins_base ----
    T_ins_base, T_base_ins = xyzrpy_deg_to_se3(
        (args.ext_x, args.ext_y, args.ext_z),
        (args.ext_roll, args.ext_pitch, args.ext_yaw))

    # ---- 3. 编码器角度插值到惯导时刻 ----
    theta, valid, stats = interp_encoder_deg(
        ins_ts, enc_ts, enc_deg, args.max_gap, fill_head=args.fill_head)

    if not valid.any():
        print("[警告] 没有任何惯导帧落在编码器的有效时间范围内，未输出文件。",
              file=sys.stderr)
        return 1

    # ---- 4. 组合位姿 ----
    xyz_ch, quat_ch, yaw_ch = build_chassis_poses(
        ins_xyz, ins_quat, theta, T_ins_base)

    ts_str_out = [s for s, ok in zip(ins_ts_str, valid) if ok]
    xyz_out, quat_out, yaw_out = xyz_ch[valid], quat_ch[valid], yaw_ch[valid]

    # ---- 5. 写文件 ----
    write_tum(out_path, ts_str_out, xyz_out, quat_out)

    # ---- 6. 统计与校验 ----
    R_wi = Rotation.from_quat(ins_quat).as_matrix()
    yaw_ins = np.degrees(np.arctan2(R_wi[:, 1, 0], R_wi[:, 0, 0]))
    path_len = float(np.sum(np.linalg.norm(np.diff(xyz_out[:, :2], axis=0), axis=1)))

    print("========== 输入 ==========")
    print(f"惯导   {ins_path}")
    print(f"       {len(ins_ts)} 帧, {ins_ts[0]:.3f} ~ {ins_ts[-1]:.3f} "
          f"({ins_ts[-1] - ins_ts[0]:.1f}s, {rate_of(ins_ts):.1f}Hz)")
    print(f"       yaw 范围 [{yaw_ins.min():.2f}, {yaw_ins.max():.2f}] deg")
    print(f"编码器 {enc_path}  (第 {args.angle_col} 列, 0 起索引)")
    print(f"       {len(enc_ts)} 帧, {enc_ts[0]:.3f} ~ {enc_ts[-1]:.3f} "
          f"({enc_ts[-1] - enc_ts[0]:.1f}s, {rate_of(enc_ts):.1f}Hz)")
    print(f"       角度范围 [{enc_deg.min():.3f}, {enc_deg.max():.3f}] deg "
          f"(sign={args.angle_sign:+g}, offset={args.angle_offset_deg:+g})")

    print("\n========== 外参 T_ins_base ==========")
    print(f"xyz = ({args.ext_x}, {args.ext_y}, {args.ext_z}) m, "
          f"rpy = ({args.ext_roll}, {args.ext_pitch}, {args.ext_yaw}) deg")
    np.set_printoptions(suppress=True, precision=8)
    print(T_ins_base)

    print("\n========== 时间对齐 ==========")
    if stats["head"]:
        tag = "已用编码器首值填充" if args.fill_head else "已丢弃"
        print(f"[警告] 前 {stats['head']} 帧惯导早于编码器起始时刻"
              f"（{enc_ts[0] - ins_ts[0]:.2f}s），{tag}")
    if stats["tail"]:
        print(f"[警告] 后 {stats['tail']} 帧惯导晚于编码器结束时刻，已丢弃")
    if stats["gap"]:
        print(f"[警告] {stats['gap']} 帧落在超过 {args.max_gap}s 的编码器空洞里，已丢弃")
    print(f"保留 {int(valid.sum())} / {len(ins_ts)} 帧")

    print("\n========== 底盘轨迹 ==========")
    print(f"插值后回转角范围 [{theta[valid].min():.3f}, {theta[valid].max():.3f}] deg")
    print(f"底盘 yaw 范围    [{yaw_out.min():.2f}, {yaw_out.max():.2f}] deg")
    print(f"底盘原点 XY 里程 {path_len:.2f} m")

    chk = heading_vs_travel(xyz_out, yaw_out, args.valid_win, args.valid_min_dist)
    if chk is None:
        print("[跳过] 位移不足，无法用行进方向校验底盘航向")
    else:
        fwd, rev, n = chk
        best, name = (fwd, "正行") if fwd <= rev else (rev, "倒行")
        print(f"航向校验（{n} 个有位移样本）: 与行进方向中位误差 "
              f"正行 {fwd:.2f} deg / 倒行 {rev:.2f} deg -> {name} {best:.2f} deg")
        if best > 5.0:
            print("[警告] 校验误差偏大，请检查外参、--angle-sign 或 --angle-col")

    print(f"\n[完成] {out_path}  ({len(ts_str_out)} 帧)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
