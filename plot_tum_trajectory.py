#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plot one or more TUM trajectories in XY, with heading arrows derived from quaternions.

TUM format:
    timestamp x y z qx qy qz qw

Edit the CONFIG section below to set input paths directly.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class TrajSpec:
    path: str
    label: str | None = None
    color: str | None = None
    arrow_step: int = 25


# ======================== 配置区 ========================
TRAJECTORIES = [
    TrajSpec(
        path="/home/chenchen/code/parse_bag/parse_res_0826/"
             "rosbag2_2026_08_26-03_56_05_parsed_res/chassis_utm.txt",
        label="chassis_utm",
        color="tab:blue",
        arrow_step=50,
    ),
]

OUTPUT_PNG = None  # 例如 "/home/chenchen/code/parse_bag/output/tum_traj.png"
FIGSIZE = (9, 8)
ARROW_LENGTH_RATIO = 0.08
MIN_ARROW_COUNT = 8
MPLCONFIGDIR = Path("/tmp/plot_tum_trajectory_mpl")
# =======================================================


def read_tum(path: Path):
    pts = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                vals = [float(v) for v in parts[:8]]
            except ValueError:
                continue
            pts.append(vals)
    if not pts:
        return np.empty((0, 8), dtype=np.float64)
    return np.asarray(pts, dtype=np.float64)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def pick_arrow_indices(n: int, step: int):
    if n <= 0:
        return np.array([], dtype=np.int64)
    step = max(1, int(step))
    idx = np.arange(0, n, step, dtype=np.int64)
    if idx.size == 0:
        idx = np.array([0], dtype=np.int64)
    if idx[-1] != n - 1 and n >= 2:
        idx = np.unique(np.append(idx, n - 1))
    return idx


def plot_trajectories(trajs):
    MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if not has_display:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=FIGSIZE)

    all_x = []
    all_y = []

    for spec in trajs:
        path = Path(spec.path)
        data = read_tum(path)
        if data.shape[0] == 0:
            print(f"[skip] {path} has no valid TUM rows")
            continue
        if data.shape[1] < 8:
            print(f"[skip] {path} has insufficient columns")
            continue

        xs = data[:, 1]
        ys = data[:, 2]
        yaws = np.array([quat_to_yaw(*row[4:8]) for row in data], dtype=np.float64)
        idx = pick_arrow_indices(len(xs), spec.arrow_step)

        label = spec.label or path.stem
        color = spec.color

        ax.plot(xs, ys, "-", lw=1.5, color=color, label=label)
        ax.scatter(xs, ys, s=8, color=color, alpha=0.55)

        all_x.append(xs)
        all_y.append(ys)

        if len(idx) >= MIN_ARROW_COUNT or len(xs) < MIN_ARROW_COUNT:
            span = max(float(np.ptp(xs)), float(np.ptp(ys)), 1.0)
            arrow_len = span * ARROW_LENGTH_RATIO
            ax.quiver(
                xs[idx], ys[idx],
                np.cos(yaws[idx]), np.sin(yaws[idx]),
                angles="xy",
                scale_units="xy",
                scale=1.0 / arrow_len,
                width=0.0025,
                color=color,
                alpha=0.85,
            )
        else:
            print(f"[warn] {path} too short for dense arrows, showing only points")

        print(
            f"{label}: {len(xs)} poses, "
            f"x=[{xs.min():.3f}, {xs.max():.3f}], "
            f"y=[{ys.min():.3f}, {ys.max():.3f}]"
        )

    if not all_x:
        raise SystemExit("No valid trajectories to plot.")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x / easting (m)")
    ax.set_ylabel("y / northing (m)")
    ax.grid(True, ls="--", alpha=0.35)
    ax.legend()
    ax.set_title("TUM trajectory with heading arrows")
    fig.tight_layout()

    if OUTPUT_PNG:
        out_path = Path(OUTPUT_PNG)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        print(f"saved: {out_path}")

    if has_display and plt.get_backend().lower() != "agg":
        plt.show()
    plt.close(fig)


def main():
    plot_trajectories(TRAJECTORIES)


if __name__ == "__main__":
    main()
