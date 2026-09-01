#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Estimate the swing center from a TUM trajectory.

Input:
    timestamp tx ty tz qx qy qz qw

Assumption:
    The upper structure rotates around a vertical axis while the chassis stays fixed.
    In that case the antenna path projected to XY is close to a circle.

Output:
    Circle center in the same frame as the input trajectory, plus the vector from
    the first pose to the center.
"""

from __future__ import annotations

import argparse
import os
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class CircleFitResult:
    cx: float
    cy: float
    radius: float
    rmse: float
    max_abs_err: float
    inlier_ratio: float
    num_points: int


@dataclass
class HeadingTangentResult:
    mean_rad: float
    std_rad: float
    rotation_sign: int


def read_tum_points(path: Path, x_col: int = 1, y_col: int = 2, z_col: int = 3,
                    qx_col: int = 4, qy_col: int = 5, qz_col: int = 6, qw_col: int = 7):
    pts = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if len(parts) <= max(x_col, y_col, z_col, qx_col, qy_col, qz_col, qw_col):
                continue
            try:
                ts = float(parts[0])
                x = float(parts[x_col])
                y = float(parts[y_col])
                z = float(parts[z_col])
                qx = float(parts[qx_col])
                qy = float(parts[qy_col])
                qz = float(parts[qz_col])
                qw = float(parts[qw_col])
            except ValueError:
                continue
            pts.append((ts, x, y, z, qx, qy, qz, qw))
    return pts


def fit_circle_xy(xs: np.ndarray, ys: np.ndarray):
    # Solve x^2 + y^2 + a*x + b*y + c = 0
    a_mat = np.column_stack([xs, ys, np.ones_like(xs)])
    b_vec = -(xs * xs + ys * ys)
    sol, *_ = np.linalg.lstsq(a_mat, b_vec, rcond=None)
    a, b, c = sol
    cx = -0.5 * a
    cy = -0.5 * b
    r_sq = cx * cx + cy * cy - c
    radius = math.sqrt(max(r_sq, 0.0))
    residual = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2) - radius
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    max_abs_err = float(np.max(np.abs(residual)))
    return cx, cy, radius, residual, rmse, max_abs_err


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def circular_mean(angles: np.ndarray):
    return math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))


def heading_tangent_angle(ts: np.ndarray, xs: np.ndarray, ys: np.ndarray, yaws: np.ndarray,
                          fit: CircleFitResult, mask: np.ndarray):
    radial_yaws = np.unwrap(np.arctan2(ys - fit.cy, xs - fit.cx))
    if len(radial_yaws) >= 2:
        slope = np.polyfit(ts - ts[0], radial_yaws, 1)[0]
        rotation_sign = 1 if slope >= 0.0 else -1
    else:
        rotation_sign = 1

    tangent_yaws = radial_yaws + rotation_sign * math.pi / 2.0
    angle_diff = np.array([wrap_to_pi(yaw - tangent) for yaw, tangent in zip(yaws, tangent_yaws)])
    inlier_diff = angle_diff[mask]
    mean_rad = circular_mean(inlier_diff)
    centered = np.array([wrap_to_pi(a - mean_rad) for a in inlier_diff])
    std_rad = float(np.sqrt(np.mean(centered * centered)))
    return HeadingTangentResult(mean_rad=mean_rad, std_rad=std_rad, rotation_sign=rotation_sign)


def plot_result(xs: np.ndarray, ys: np.ndarray, yaws: np.ndarray, fit: CircleFitResult,
                heading_tangent: HeadingTangentResult, out_path: str):
    mpl_config = Path("/tmp/estimate_swing_center_mpl")
    mpl_config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_config))
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    theta = np.linspace(0.0, 2.0 * math.pi, 400)
    circle_x = fit.cx + fit.radius * np.cos(theta)
    circle_y = fit.cy + fit.radius * np.sin(theta)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xs, ys, ".", ms=10, alpha=0.75, label="raw trajectory")
    step = max(1, len(xs) // 80)
    idx = np.arange(0, len(xs), step)
    arrow_len = max(0.08, fit.radius * 0.12)
    ax.quiver(
        xs[idx], ys[idx],
        np.cos(yaws[idx]), np.sin(yaws[idx]),
        angles="xy",
        scale_units="xy",
        scale=1.0 / arrow_len,
        width=0.0025,
        color="tab:green",
        alpha=0.8,
        label="heading",
    )
    ax.plot(circle_x, circle_y, "-", lw=2, label="fitted circle")
    ax.plot([fit.cx], [fit.cy], "r*", ms=14, label="center")
    ax.annotate(
        f"center\n({fit.cx:.3f}, {fit.cy:.3f}) m",
        xy=(fit.cx, fit.cy),
        xytext=(12, 12),
        textcoords="offset points",
        fontsize=9,
        color="red",
        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="red", alpha=0.85),
        arrowprops=dict(arrowstyle="->", color="red", lw=1.0),
    )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, ls="--", alpha=0.35)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_title(
        f"circle fit: r={fit.radius:.3f} m, rmse={fit.rmse:.4f} m, "
        f"heading-tangent={math.degrees(heading_tangent.mean_rad):.2f} deg"
    )
    fig.tight_layout()

    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.show()
    plt.close(fig)


def robust_circle_fit(xs: np.ndarray, ys: np.ndarray, trim_sigma: float = 3.0, max_iter: int = 5):
    mask = np.ones(len(xs), dtype=bool)
    for _ in range(max_iter):
        cx, cy, radius, residual, rmse, max_abs_err = fit_circle_xy(xs[mask], ys[mask])
        if trim_sigma <= 0:
            break
        med = float(np.median(residual))
        mad = float(np.median(np.abs(residual - med)))
        if mad < 1e-12:
            break
        sigma = 1.4826 * mad
        new_mask = np.abs(residual - med) <= trim_sigma * sigma
        if new_mask.all() or new_mask.sum() < 8:
            break
        idx = np.flatnonzero(mask)
        mask[:] = False
        mask[idx[new_mask]] = True
    cx, cy, radius, residual, rmse, max_abs_err = fit_circle_xy(xs[mask], ys[mask])
    return CircleFitResult(
        cx=float(cx),
        cy=float(cy),
        radius=float(radius),
        rmse=float(rmse),
        max_abs_err=float(max_abs_err),
        inlier_ratio=float(mask.mean()),
        num_points=int(mask.sum()),
    ), mask


def main():
    parser = argparse.ArgumentParser(description="Estimate swing center from a TUM trajectory.")
    parser.add_argument("traj_file", help="Input TUM file")
    parser.add_argument(
        "--trim-sigma",
        type=float,
        default=3.0,
        help="Robust trimming threshold in sigma. Set <=0 to disable.",
    )
    args = parser.parse_args()

    traj_path = Path(args.traj_file)
    pts = read_tum_points(traj_path)
    if len(pts) < 8:
        raise SystemExit(f"Need at least 8 valid poses, got {len(pts)}")

    ts = np.array([p[0] for p in pts], dtype=np.float64)
    xs = np.array([p[1] for p in pts], dtype=np.float64)
    ys = np.array([p[2] for p in pts], dtype=np.float64)
    zs = np.array([p[3] for p in pts], dtype=np.float64)
    yaws = np.array([quat_to_yaw(p[4], p[5], p[6], p[7]) for p in pts], dtype=np.float64)

    fit, mask = robust_circle_fit(xs, ys, trim_sigma=args.trim_sigma)
    heading_tangent = heading_tangent_angle(ts, xs, ys, yaws, fit, mask)
    z_center = float(np.mean(zs[mask]))

    x0, y0, z0 = xs[0], ys[0], zs[0]
    rel = {
        "x": fit.cx - x0,
        "y": fit.cy - y0,
        "z": z_center - z0,
    }
    text = (
        f"center_world: ({fit.cx:.6f}, {fit.cy:.6f}, {z_center:.6f}) m\n"
        f"center_relative_to_first_pose: ({rel['x']:.6f}, {rel['y']:.6f}, {rel['z']:.6f}) m\n"
        f"radius: {fit.radius:.6f} m\n"
        f"rmse: {fit.rmse:.6f} m\n"
        f"max_abs_error: {fit.max_abs_err:.6f} m\n"
        f"inliers: {mask.sum()}/{len(mask)}\n"
        f"heading_minus_tangent: {math.degrees(heading_tangent.mean_rad):.6f} deg\n"
        f"heading_minus_tangent_std: {math.degrees(heading_tangent.std_rad):.6f} deg\n"
        f"rotation_direction: {'ccw' if heading_tangent.rotation_sign > 0 else 'cw'}"
    )

    print(text)

    plot_path = traj_path.with_name(f"{traj_path.stem}_swing_center.png")
    plot_result(xs, ys, yaws, fit, heading_tangent, out_path=str(plot_path))
    print(f"plot saved: {plot_path}")


if __name__ == "__main__":
    main()
