#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 GNSS 位置(gps_pos)与航向(gps_hdg)按时间戳匹配，输出 UTM 坐标系下的 TUM 轨迹。

输入（均由 parse_bag.py 生成）：
  gps_pos.txt : timestamp lat(deg) lon(deg) alt(m)                  （GnssPosMsg / Gpgga）
  gps_hdg.txt : timestamp hdg_type baseline heading(deg) pitch(deg) （GnssHdgMsg）

输出 TUM 文件，每行：
  timestamp easting northing height qx qy qz qw
  - 位置：经纬度转 UTM easting/northing，高程直接用 alt；
  - 姿态：roll=0，pitch 取 hdg 的 pitch，yaw 取 hdg 的 heading。
    GnssHdgMsg 的 heading 定义是 "counterclockwise from east [deg]"（即 ENU 系 yaw），
    先度转弧度，再经 ENUYawToUTMYaw 修正 UTM 网格收敛角，最后 RPY2mat + 四元数。

用法：
  python3 combine_pos_hdg_tum.py --dir output
  python3 combine_pos_hdg_tum.py --pos output/gps_pos.txt --hdg output/gps_hdg.txt \
      --out output/gps_traj_utm.txt
"""

import argparse
from pathlib import Path

import utils


def main():
    ap = argparse.ArgumentParser(description="组合 GNSS 位置与航向 -> UTM TUM 轨迹")
    ap.add_argument("--dir", default=None,
                    help="目录，自动查找 gps_pos.txt 与 gps_hdg.txt")
    ap.add_argument("--pos", default=None, help="位置文件(llh)路径")
    ap.add_argument("--hdg", default=None, help="航向文件路径")
    ap.add_argument("--out", default=None, help="输出 TUM 文件路径")
    ap.add_argument("--max-dt", type=float, default=2.0,
                    help="匹配时允许的最大时间差(秒)，默认 2.0")
    args = ap.parse_args()

    pos, hdg, out = args.pos, args.hdg, args.out

    if args.dir:
        d = Path(args.dir)
        if pos is None:
            # 位置文件：优先 gps_pos.txt，其次 gps_pose.txt / gps_gpgga.txt
            for name in ("gps_pos.txt", "gps_pose.txt", "gps_gpgga.txt"):
                cand = d / name
                if cand.exists():
                    pos = str(cand)
                    break
        if hdg is None:
            cand = d / "gps_hdg.txt"
            if cand.exists():
                hdg = str(cand)
        if out is None:
            out = str(d / "gps_traj_utm.txt")

    if not pos or not hdg:
        ap.error("需要 --dir，或同时给出 --pos 与 --hdg")

    pos = str(pos)
    hdg = str(hdg)
    if out is None:
        out = str(Path(pos).with_name("gps_traj_utm.txt"))

    n = utils.combine_pos_hdg_to_tum(pos, hdg, out, max_dt=args.max_dt)
    print(f"生成 TUM: {out} ({n} 帧)")


if __name__ == "__main__":
    main()
