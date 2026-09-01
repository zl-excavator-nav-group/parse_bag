#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单独绘制 *_timing_analysis.csv 的时序诊断图。

读取 parse_bag 时序分析导出的 <topic>_timing_analysis.csv（列：
t_h, t_r, diff_ms, delta_h, delta_r, is_out_of_order），把
diff_ms、delta_h、delta_r、is_out_of_order 四列画成子图，并弹出窗口显示。

运行：
    python3 tools/plot_timing_analysis.py [csv 路径]
不给路径时使用下方 CSV_FILE 常量。

说明：
    - diff_ms      直接画（单位毫秒）；
    - delta_h/delta_r 文件里是秒，这里换算成毫秒显示（与报告的"间隔(ms)"一致）；
    - is_out_of_order 是 0/1 标志，只把 ==1 的点用红点标出。
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# ======================== 配置区 ========================
# 默认要画的 CSV 文件（命令行第一个参数可覆盖）
CSV_FILE = ("/home/chenchen/code/parse_bag/output_0827/rosbag2_2026_08_27-08_53_39_parsed_res/timing_report/excavator_swing_encoder_state_timing_analysis.csv")

# 采样/到达间隔、diff 的纵轴显示上限（毫秒），超过的极值会被裁剪以便看细节；None 表示不裁剪
YMAX_MS = None

# 是否使用相对时间做横轴（True：各列都从 0 秒开始；False：用原始时间戳）
USE_RELATIVE_TIME = True
# ================================================================


def _setup_cjk_font():
    """配置 matplotlib 中文字体，避免标题中文显示为方框。"""
    matplotlib.rcParams["axes.unicode_minus"] = False
    candidates = [
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
    ]
    for path in candidates:
        if Path(path).exists():
            try:
                fm.fontManager.addfont(path)
                name = fm.FontProperties(fname=path).get_name()
                matplotlib.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
                return name
            except Exception:
                continue
    known = {f.name for f in fm.fontManager.ttflist}
    for name in ("WenQuanYi Micro Hei", "Noto Sans CJK SC", "Noto Sans CJK JP",
                 "Droid Sans Fallback", "AR PL UMing CN", "SimHei", "Microsoft YaHei"):
        if name in known:
            matplotlib.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            return name
    return None


def load_csv(csv_path):
    """读取 CSV，返回各列 numpy 数组。"""
    df = pd.read_csv(csv_path)
    need = {"t_h", "t_r", "diff_ms", "delta_h", "delta_r", "is_out_of_order"}
    missing = need - set(df.columns)
    if missing:
        raise ValueError(f"CSV 缺少列：{sorted(missing)}（实际列：{list(df.columns)}）")

    t_h = df["t_h"].to_numpy(dtype=float)
    t_r = df["t_r"].to_numpy(dtype=float)
    diff_ms = df["diff_ms"].to_numpy(dtype=float)
    delta_h_s = df["delta_h"].to_numpy(dtype=float)   # 秒，可能有 NaN
    delta_r_s = df["delta_r"].to_numpy(dtype=float)   # 秒，可能有 NaN
    ooo = df["is_out_of_order"].to_numpy(dtype=float)  # 0/1

    return t_h, t_r, diff_ms, delta_h_s, delta_r_s, ooo


def _add_hstat_lines(ax, values):
    """在 ax 上叠加均值 / p95 / p99 三条水平参考线（自动忽略 NaN）。"""
    v = np.asarray(values, dtype=float)
    v = v[~np.isnan(v)]
    if v.size == 0:
        return
    mean = float(np.mean(v))
    p95 = float(np.percentile(v, 95))
    p99 = float(np.percentile(v, 99))
    ax.axhline(mean, color="green", linestyle="--", linewidth=1, label=f"mean = {mean:.3f}")
    ax.axhline(p95, color="orange", linestyle="--", linewidth=1, label=f"p95 = {p95:.3f}")
    ax.axhline(p99, color="red", linestyle="--", linewidth=1, label=f"p99 = {p99:.3f}")


def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(CSV_FILE)
    if not csv_path.exists():
        print(f"[错误] CSV 文件不存在：{csv_path}", file=sys.stderr)
        sys.exit(1)

    _setup_cjk_font()
    t_h, t_r, diff_ms, delta_h_s, delta_r_s, ooo = load_csv(csv_path)

    topic = csv_path.stem.replace("_timing_analysis", "")

    # 横轴：相对时间或原始时间戳
    if USE_RELATIVE_TIME:
        x_h = t_h - t_h[0]
        x_r = t_r - t_r[0]
        x_h_label, x_r_label = "t_h - t_h[0] (s)", "t_r - t_r[0] (s)"
    else:
        x_h, x_r = t_h, t_r
        x_h_label, x_r_label = "t_h (s)", "t_r (s)"

    # delta_h / delta_r 换算成毫秒
    delta_h_ms = delta_h_s * 1000.0
    delta_r_ms = delta_r_s * 1000.0

    fig, axs = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

    # 1) diff_ms
    axs[0].plot(x_h, diff_ms, linestyle='-', linewidth=1, marker='o', markersize=3)
    axs[0].set_ylabel("diff_ms (ms)")
    axs[0].set_title(f"{topic} - diff（接收时间 t_r − header 时间 t_h）")
    _add_hstat_lines(axs[0], diff_ms)

    # 2) delta_h
    axs[1].plot(x_h, delta_h_ms, linestyle='-', linewidth=1, marker='o', markersize=3)
    axs[1].set_ylabel("delta_h (ms)")
    axs[1].set_title(f"{topic} - 采样间隔（按 t_h 排序后相邻 header 时间差）")
    _add_hstat_lines(axs[1], delta_h_ms)

    # 3) delta_r
    axs[2].plot(x_r, delta_r_ms, linestyle='-', linewidth=1, marker='o', markersize=3)
    axs[2].set_ylabel("delta_r (ms)")
    axs[2].set_title(f"{topic} - 到达间隔（相邻接收时间差）")
    _add_hstat_lines(axs[2], delta_r_ms)

    # 4) is_out_of_order
    axs[3].plot(x_h, np.zeros_like(x_h), color="0.85", linewidth=0.5)  # 0 基线
    idx = np.where(ooo == 1)[0]
    axs[3].scatter(x_h[idx], np.ones_like(idx), s=6, color="red",
                   label=f"乱序帧数 = {len(idx)}")
    axs[3].set_ylabel("is_out_of_order")
    axs[3].set_ylim(-0.1, 1.1)
    axs[3].set_title(f"{topic} - 到达乱序（原始到达顺序下 t_h 变小的帧）")
    axs[3].legend()

    # 纵轴裁剪与坐标标注
    for i, ax in enumerate(axs):
        ax.grid(True, alpha=0.3)
        if YMAX_MS is not None:
            ax.set_ylim(top=YMAX_MS)
        if i < 3:
            ax.legend(fontsize=7)
    axs[0].set_xlabel(x_h_label)
    axs[1].set_xlabel(x_h_label)
    axs[2].set_xlabel(x_r_label)
    axs[3].set_xlabel(x_h_label)

    fig.suptitle(f"{topic} 时序诊断（{len(t_h)} 帧）", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    plt.show()


if __name__ == "__main__":
    main()
