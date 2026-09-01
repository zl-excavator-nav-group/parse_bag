#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plot TUM trajectories with heading arrows.

The trajectory file paths are configured directly in this script.
"""

import os
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
MPLCONFIGDIR = Path("/tmp/plot_tum_traj_mpl")
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

import numpy as np

sys.path.append(str(SCRIPT_DIR))
sys.path.append(str(REPO_ROOT))
import PoseData  # noqa: E402


# ======================== 配置区 ========================
TRAJ_FILES = [
    "/home/chenchen/code/parse_bag/parse_res_0826/"
    "rosbag2_2026_08_26-03_56_05_parsed_res/chassis_utm.txt",
]

TRAJ_LABELS = [
    "chassis_utm",
]

TRAJ_COLORS = [
    "tab:blue",
]

OUT_PNG = None  # 例如 "/home/chenchen/code/parse_bag/output/tum_traj.png"
SHOW_SCATTER = True
ARROW_LENGTH = 0.2
ARROW_WIDTH = 0.1
MIN_TIME_INTERVAL = 1.0  # 秒，<=0 表示不过滤
# =======================================================


def draw_arrow(axs, x, y, angle, length, width, color="blue"):
    """Draw a heading arrow at (x, y)."""
    dx = length * np.cos(angle)
    dy = length * np.sin(angle)
    axs.arrow(x, y, dx, dy, head_width=width, head_length=width * 2, fc=color, ec=color)


def plot_traj_list(ax, show_traj_list, inter_valid_list, is_scatter=False, linestyle="-",
                   labels=None, colors=None):
    for i in range(len(show_traj_list)):
        if len(show_traj_list[i]) == 0:
            continue
        valid_flag = inter_valid_list[i]
        _, x_coords, y_coords, _, all_yaws = PoseData.get_coords(show_traj_list[i])
        label = labels[i] if labels and i < len(labels) else str(i)
        color = colors[i] if colors and i < len(colors) else None

        if valid_flag == 0:
            if is_scatter:
                ax.scatter(x_coords, y_coords, color="gray", s=0.5, label=label)
                ax.text(x_coords[0], y_coords[0], label, fontsize=10)
            else:
                ax.plot(x_coords, y_coords, color="gray", linestyle=linestyle, label=label)
        else:
            if is_scatter:
                ax.scatter(x_coords, y_coords, color=color, s=8, label=label)
                ax.text(x_coords[0], y_coords[0], label, fontsize=10)
                for j in range(len(x_coords)):
                    draw_arrow(ax, x_coords[j], y_coords[j], all_yaws[j],
                               ARROW_LENGTH, ARROW_WIDTH, color=color or "blue")
            else:
                ax.plot(x_coords, y_coords, color=color, linestyle=linestyle, label=label)


def load_traj_list(file_list, min_time_interval=5.0):
    traj_list = []
    valid_list = []
    for path in file_list:
        traj_list.append(PoseData.read_trajectory_file(path, min_time_interval=min_time_interval))
        valid_list.append(1)
    return traj_list, valid_list


def main():
    MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    traj_list, valid_list = load_traj_list(TRAJ_FILES, min_time_interval=MIN_TIME_INTERVAL)

    fig, ax = plt.subplots(figsize=(9, 8))
    plot_traj_list(
        ax,
        traj_list,
        valid_list,
        is_scatter=SHOW_SCATTER,
        labels=TRAJ_LABELS,
        colors=TRAJ_COLORS,
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, ls="--", alpha=0.35)
    ax.legend()
    ax.set_title("TUM trajectory with heading arrows")
    fig.tight_layout()

    if OUT_PNG:
        out_path = Path(OUT_PNG)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=200, bbox_inches="tight")
        print(f"saved: {out_path}")

    if has_display and plt.get_backend().lower() != "agg":
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
