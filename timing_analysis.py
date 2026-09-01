#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多传感器时序 / 时间同步分析脚本。

输入：parse_bag.py 为每个 topic 导出的 *_timing.txt，每行三列（空格分隔）：
    t_h        header.stamp 时间戳（秒，浮点，含纳秒）
    t_r        消息接收时间（秒，浮点，含纳秒）
    diff_ms    t_r - t_h（毫秒）
此外跨 IMU 同步会读取同名的 IMU 数据 txt（列：t_h acc_x acc_y acc_z gyro_x gyro_y gyro_z），
先把各 IMU 转到统一 base_link 坐标系，再用角速度波形做互相关估计时间偏移。

功能（对应 plan.md 的模块 2~5）：
- 单话题时序分析：采样间隔 delta_h、到达间隔 delta_r、到达乱序、diff 延迟统计、丢帧估计；
- 跨话题时间同步：互相关 + 波峰波谷匹配估计 offset，滑动窗口观察漂移；
- 全局异常检测：按阈值输出异常事件列表；
- 报告导出：Markdown 汇总报告 + 每话题 CSV + 跨 IMU 对比 CSV + PNG 图 + 汇总 PDF。

用法（无需命令行参数，全部配置在 timing_analysis_config.yaml）：
    python3 timing_analysis.py

依赖：numpy、pyyaml、scipy（跨话题互相关用）、matplotlib（绘图）、reportlab（PDF 导出）。
"""

import csv
import sys
from pathlib import Path
import numpy as np
import yaml

# 复用 imu_to_base_link.py 的 IMU -> base_link 外参旋转（与本脚本同目录）
import imu_to_base_link

CONFIG_PATH = Path(__file__).with_name("timing_analysis_config.yaml")

_NAN = float("nan")

DEFAULTS = {
    "input_dir": "",
    "output_dir": "",
    "topic_filter": [],
    "sensor_freq_hz": {},
    "thresholds": {
        "diff_spike_ms": 80.0,
        "out_of_order_ratio_max": 0.01,
        "gap_factor": 10.0,
        "burst_eps_ms": 0.5,
        "burst_min_count": 5,
        "offset_warn_ms": 20.0,
    },
    "sync": {
        "topics": ["gps_imu_raw", "livox_imu_192_168_1_158"],
        "axis": "z",
        "max_offset_s": 1.0,
        "sample_dt": 0.01,
    },
    "plot": {"enabled": True},
}


# ======================== 配置加载 ========================

def _deep_merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v


def load_config(path=CONFIG_PATH):
    """读取 YAML 配置并叠加到默认值上；配置文件缺失则用默认值。"""
    import copy
    cfg = copy.deepcopy(DEFAULTS)
    if Path(path).exists():
        with open(path, encoding="utf-8") as f:
            loaded = yaml.safe_load(f) or {}
        _deep_merge(cfg, loaded)
    else:
        print(f"[警告] 未找到配置文件 {path}，使用内置默认值。", file=sys.stderr)
    return cfg


# ======================== 数据加载与校验 ========================

def read_numeric_rows(path):
    """逐行读取空格分隔数值，跳过表头/非法行；返回 (list[list[float]], 跳过行数)。"""
    rows = []
    skipped = 0
    with open(path, encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            parts = s.split()
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                skipped += 1
    return rows, skipped


def parse_timing_file(path):
    """解析 *_timing.txt -> (t_h, t_r, diff_ms, skipped)；无有效数据返回 None。"""
    rows, skipped = read_numeric_rows(path)
    arr = np.asarray(rows, dtype=float)
    if arr.size == 0 or arr.shape[1] < 3:
        return None
    return arr[:, 0], arr[:, 1], arr[:, 2], skipped


def discover_topics(input_dir, topic_filter):
    """扫描 input_dir 下所有 *_timing.txt，返回 {topic: 路径}。"""
    d = Path(input_dir)
    if not d.is_dir():
        raise SystemExit(f"输入目录不存在：{input_dir}")
    files = sorted(d.glob("*_timing.txt"))
    topics = {}
    for p in files:
        topic = p.name.replace("_timing.txt", "")
        if topic_filter and not any(k.lower() in topic.lower() for k in topic_filter):
            continue
        topics[topic] = p
    return topics


def theoretical_period(topic, sensor_freq_hz):
    """按 topic 名关键字匹配理论频率 -> 周期（秒）；未配置返回 None。"""
    tl = topic.lower()
    for key, hz in sensor_freq_hz.items():
        if str(key).lower() in tl and hz and hz > 0:
            return 1.0 / float(hz)
    return None


# ======================== 单话题时序分析 ========================

def _consecutive_runs(mask):
    """布尔数组中连续 True 段的 (start, length) 列表。"""
    runs = []
    n = len(mask)
    i = 0
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            runs.append((i, j - i))
            i = j
        else:
            i += 1
    return runs


def _stats(a):
    if len(a) == 0:
        return {"mean": _NAN, "std": _NAN, "min": _NAN, "max": _NAN}
    return {"mean": float(np.mean(a)), "std": float(np.std(a)),
            "min": float(np.min(a)), "max": float(np.max(a))}


def analyze_single_topic(topic, t_h, t_r, diff_ms, period, thr):
    """对单个 topic 做时序分析，返回 (结果 dict, 异常事件列表)。

    结果 dict 除各项统计外，还附带 delta_h_row/delta_r_row/ooo 三个逐帧数组（供 CSV 导出）。
    异常事件为 (类型, 时间, 数值) 三元组。
    """
    n = len(t_h)
    res = {"topic": topic, "n": n, "period": period}
    anomalies = []

    # 按 header 时间排序（理想采样顺序）
    order = np.argsort(t_h, kind="stable")
    sh = t_h[order]
    delta_h = np.diff(sh) if n > 1 else np.array([])

    # 把 sorted 的 delta_h 映射回原始到达顺序（供 CSV）
    delta_h_row = np.full(n, _NAN)
    if n > 1:
        delta_h_row[order[1:]] = delta_h

    # 到达间隔（原始到达顺序）
    delta_r = np.diff(t_r) if n > 1 else np.array([])
    delta_r_row = np.full(n, _NAN)
    if n > 1:
        delta_r_row[1:] = delta_r

    # 到达乱序：原始顺序下 t_h 变小
    ooo = np.zeros(n, dtype=bool)
    if n > 1:
        ooo[1:] = t_h[1:] < t_h[:-1]

    res["delta_h_row"] = delta_h_row
    res["delta_r_row"] = delta_r_row
    res["ooo"] = ooo

    if n == 0:
        res["valid"] = False
        return res, anomalies
    res["valid"] = True

    # 理论周期：未配置则用 median(delta_h) 自动估计
    med_dh = float(np.median(delta_h)) if n > 1 else _NAN
    res["delta_h_median"] = med_dh
    if period is None:
        period = med_dh if (n > 1 and med_dh > 0) else None
    res["period"] = period

    res["delta_h"] = _stats(delta_h)
    res["delta_r"] = _stats(delta_r)

    # 断流 / 爆帧 / 接收断流（都基于理论周期）
    if period and period > 0 and n > 1:
        gap_thr = thr["gap_factor"] * period

        gap_idx = np.where(delta_h > gap_thr)[0]
        res["sample_gap_count"] = int(len(gap_idx))
        for i in gap_idx[:50]:
            anomalies.append(("采样断流", float(sh[i + 1]), float(delta_h[i]) * 1000.0))

        burst_eps = thr["burst_eps_ms"] / 1000.0
        groups = [g for g in _consecutive_runs(delta_h < burst_eps)
                  if g[1] >= thr["burst_min_count"]]
        res["burst_group_count"] = len(groups)
        res["burst_max_len"] = max([g[1] for g in groups], default=0)
        for start, length in groups[:20]:
            anomalies.append(("爆帧", float(sh[start]), float(length)))

        # 接收断流：到达间隔过大，且并非由采样断流（源端停止采样）引起。
        # 若这对相邻到达的 header 时间本身也断开（> gap_thr），说明是源头没发，已归为
        # 采样断流，此处不再重复诊断为接收断流。
        arr_delta_h = np.diff(t_h)  # 原始到达顺序下，相邻两帧的 header 时间差
        arr_gap_idx = np.where((delta_r > gap_thr) & (arr_delta_h <= gap_thr))[0]
        res["arrival_gap_count"] = int(len(arr_gap_idx))
        for i in arr_gap_idx[:50]:
            anomalies.append(("接收断流", float(t_r[i + 1]), float(delta_r[i]) * 1000.0))
    else:
        res["sample_gap_count"] = res["burst_group_count"] = res["burst_max_len"] = 0
        res["arrival_gap_count"] = 0

    # 乱序统计
    ooo_count = int(ooo.sum())
    res["out_of_order_count"] = ooo_count
    res["out_of_order_ratio"] = float(ooo_count / max(n - 1, 1))
    res["out_of_order_times"] = [float(t_h[i]) for i in np.where(ooo)[0][:20]]

    # diff 延迟统计
    res["diff"] = _stats(diff_ms)
    res["diff_p50"] = float(np.percentile(diff_ms, 50))
    res["diff_p95"] = float(np.percentile(diff_ms, 95))
    res["diff_p99"] = float(np.percentile(diff_ms, 99))

    # 延迟尖峰
    spike = diff_ms > thr["diff_spike_ms"]
    res["diff_spike_count"] = int(spike.sum())
    for i in np.where(spike)[0][:20]:
        anomalies.append(("延迟尖峰", float(t_h[i]), float(diff_ms[i])))

    # diff 漂移（线性回归斜率，单位 ms/s）
    if n > 1 and np.ptp(t_h) > 0:
        res["diff_drift_ms_per_s"] = float(np.polyfit(t_h, diff_ms, 1)[0])
    else:
        res["diff_drift_ms_per_s"] = _NAN

    # 丢帧率估计（bag 录制视角）
    if period and period > 0 and n > 1:
        theo = (sh[-1] - sh[0]) / period + 1
        res["loss_rate"] = max(0.0, 1.0 - n / theo)
    else:
        res["loss_rate"] = _NAN

    # 乱序占比超限告警
    if res["out_of_order_ratio"] > thr["out_of_order_ratio_max"]:
        anomalies.append(("到达乱序超限", float(t_h[0]), res["out_of_order_ratio"]))

    return res, anomalies


# ======================== IMU 信号加载 + base_link 变换 ========================

def load_imu_signal(topic, timing_path):
    """加载同名 IMU 数据 txt，转 base_link，返回 (t_h, acc_b, gyro_b) 或 None。

    非 7 列 IMU 格式 / 数据文件缺失 / 长度不一致时返回 None（走时间戳粗同步）。
    """
    data_path = timing_path.with_name(topic + ".txt")
    if not data_path.exists():
        return None

    rows, _ = read_numeric_rows(data_path)
    if not rows:
        return None
    widths = {len(r) for r in rows}
    if widths != {7}:  # IMU 数据应为 7 列：t_h + acc3 + gyro3
        return None

    arr = np.asarray(rows, dtype=float)
    t_h_data = arr[:, 0]  # 数据文件第 0 列现为 header 时间 t_h
    acc = arr[:, 1:4]
    gyro = arr[:, 4:7]

    parsed = parse_timing_file(timing_path)
    if parsed is None:
        return None
    t_h, _, _, _ = parsed

    # 两者由 parse_bag.py 同一循环写出、行对齐；行数不一致则放弃互相关
    if len(t_h) != len(t_h_data):
        print(f"[警告] {topic}: timing({len(t_h)}) 与数据({len(t_h_data)}) 行数不一致，"
              f"改用时间戳粗同步。", file=sys.stderr)
        return None

    # base_link 旋转（复用 imu_to_base_link.py 的外参）
    hit = imu_to_base_link.match_extrinsic(topic, imu_to_base_link.IMU_EXTRINSIC_DEG)
    if hit is None:
        print(f"[警告] {topic}: 未在 imu_to_base_link.IMU_EXTRINSIC_DEG 配置外参，"
              f"按单位阵处理（不旋转）。", file=sys.stderr)
        R = np.eye(3)
    else:
        R = np.array(hit[1], dtype=float)

    acc_b = acc @ R.T
    gyro_b = gyro @ R.T
    return t_h, acc_b, gyro_b


def select_axis(gyro_b, axis):
    """从 (N,3) 角速度中取指定轴（或范数）作为互相关信号。"""
    if axis == "norm":
        return np.linalg.norm(gyro_b, axis=1)
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    return gyro_b[:, idx].astype(float)


# ======================== 跨话题时间同步 ========================

def _resample_uniform(ts, vals, dt):
    if len(ts) < 2:
        return np.array([]), np.array([])
    new_ts = np.arange(ts[0], ts[-1], dt)
    from scipy import interpolate
    f = interpolate.interp1d(ts, vals, kind="linear", fill_value="extrapolate")
    return new_ts, f(new_ts)


def cross_correlation_offset(ts1, g1, ts2, g2, max_offset_s, sample_dt):
    """互相关估计时间偏移（参考 imu_time_offset.estimate_offset_cross_correlation）。

    返回 (offset_sec, (lags_sec, corr_norm))；offset_sec 正值表示 topic2 相对 topic1 滞后。
    """
    from scipy import signal
    t_start = max(ts1[0], ts2[0])
    t_end = min(ts1[-1], ts2[-1])
    if t_end - t_start < 2 * max_offset_s:
        raise ValueError("两个 IMU 数据重叠时间段太短，无法互相关")

    nt1, ng1 = _resample_uniform(ts1, g1, sample_dt)
    nt2, ng2 = _resample_uniform(ts2, g2, sample_dt)
    m1 = (nt1 >= t_start) & (nt1 <= t_end)
    m2 = (nt2 >= t_start) & (nt2 <= t_end)
    s1, s2 = ng1[m1], ng2[m2]
    min_len = min(len(s1), len(s2))
    s1, s2 = s1[:min_len], s2[:min_len]
    s1 = s1 - np.mean(s1)
    s2 = s2 - np.mean(s2)

    corr = signal.correlate(s1, s2, mode="full")
    lags = signal.correlation_lags(len(s1), len(s2), mode="full") * sample_dt
    valid = np.abs(lags) <= max_offset_s
    corr_v, lags_v = corr[valid], lags[valid]
    peak = int(np.argmax(corr_v))
    offset_sec = float(lags_v[peak])

    norm = np.sqrt(np.sum(s1 ** 2) * np.sum(s2 ** 2))
    corr_norm = corr_v / norm if norm > 0 else corr_v
    return offset_sec, (lags_v, corr_norm)


def peak_matching_offset(ts1, g1, ts2, g2, sample_dt):
    """波峰波谷匹配估计偏移（参考 imu_time_offset.estimate_offset_peak_matching）。

    返回 (median_sec, mean_sec, std_sec, matched_pairs)。
    """
    from scipy import signal
    t_start = max(ts1[0], ts2[0])
    t_end = min(ts1[-1], ts2[-1])

    nt1, ng1 = _resample_uniform(ts1, g1, sample_dt)
    nt2, ng2 = _resample_uniform(ts2, g2, sample_dt)
    m1 = (nt1 >= t_start) & (nt1 <= t_end)
    m2 = (nt2 >= t_start) & (nt2 <= t_end)
    nt1, ng1 = nt1[m1], ng1[m1]
    nt2, ng2 = nt2[m2], ng2[m2]

    sos = signal.butter(4, 5.0, btype="low", fs=1.0 / sample_dt, output="sos")
    f1 = signal.sosfiltfilt(sos, ng1)
    f2 = signal.sosfiltfilt(sos, ng2)

    dist = int(0.5 / sample_dt)
    min_abs = 0.02
    p1, _ = signal.find_peaks(f1, prominence=np.std(f1) * 3.0, distance=dist)
    v1, _ = signal.find_peaks(-f1, prominence=np.std(f1) * 3.0, distance=dist)
    p2, _ = signal.find_peaks(f2, prominence=np.std(f2) * 3.0, distance=dist)
    v2, _ = signal.find_peaks(-f2, prominence=np.std(f2) * 3.0, distance=dist)
    p1 = p1[np.abs(f1[p1]) > min_abs]
    v1 = v1[np.abs(f1[v1]) > min_abs]
    p2 = p2[np.abs(f2[p2]) > min_abs]
    v2 = v2[np.abs(f2[v2]) > min_abs]

    time_diffs = []
    for t in nt1[p1]:
        if len(p2) == 0:
            break
        j = np.argmin(np.abs(nt2[p2] - t))
        d = nt2[p2][j] - t
        if abs(d) < 0.05:
            time_diffs.append(d)
    for t in nt1[v1]:
        if len(v2) == 0:
            break
        j = np.argmin(np.abs(nt2[v2] - t))
        d = nt2[v2][j] - t
        if abs(d) < 0.05:
            time_diffs.append(d)

    if not time_diffs:
        raise ValueError("未找到可匹配的波峰/波谷对")
    time_diffs = np.asarray(time_diffs)
    return (float(np.median(time_diffs)), float(np.mean(time_diffs)),
            float(np.std(time_diffs)), int(len(time_diffs)))


def timestamp_coarse_offset(t_hA, t_rA, t_hB, t_rB):
    """时间戳层面粗同步：按接收时间(同一 bag 时钟)就近匹配，取 t_hA - t_hB 中位数。

    仅作怀疑线索：该值包含两传感器传输延迟差、且受采样密度影响，不等于真实
    波形偏移（真实偏移由互相关估计）。
    """
    if len(t_hA) == 0 or len(t_hB) == 0:
        return _NAN
    ob = np.argsort(t_rB)
    t_rB_s = t_rB[ob]
    t_hB_s = t_hB[ob]
    diffs = []
    step = max(1, len(t_hA) // 2000)  # 下采样以加速
    for i in range(0, len(t_hA), step):
        j = int(np.searchsorted(t_rB_s, t_rA[i]))
        j = min(j, len(t_rB_s) - 1)
        if j > 0 and abs(t_rB_s[j - 1] - t_rA[i]) < abs(t_rB_s[j] - t_rA[i]):
            j -= 1
        diffs.append(t_hA[i] - t_hB_s[j])
    return float(np.median(diffs))


def cross_sensor_out_of_order(t_hA, t_rA, t_hB, t_rB):
    """跨传感器到达乱序计数（plan.md 3.4）：A 采样更早但接收时间晚于 B 的次数。"""
    ob = np.argsort(t_hB)
    t_hB_s, t_rB_s = t_hB[ob], t_rB[ob]
    count = 0
    for i in range(len(t_hA)):
        j = int(np.searchsorted(t_hB_s, t_hA[i]))
        j = min(j, len(t_hB_s) - 1)
        if j > 0 and abs(t_hB_s[j - 1] - t_hA[i]) < abs(t_hB_s[j] - t_hA[i]):
            j = j - 1
        if t_hA[i] < t_hB_s[j] and t_rA[i] > t_rB_s[j]:
            count += 1
    return count


def sliding_window_offsets(ta, ga, tb, gb, max_offset_s, sample_dt, n_windows=5):
    """分段重算互相关 offset，观察是否漂移；返回 [(w0, w1, offset_ms), ...]。"""
    lo = max(ta[0], tb[0])
    hi = min(ta[-1], tb[-1])
    span = hi - lo
    if span < 4 * max_offset_s:
        return []
    offs = []
    for k in range(n_windows):
        w0 = lo + span * k / n_windows
        w1 = lo + span * (k + 1) / n_windows
        m1 = (ta >= w0) & (ta <= w1)
        m2 = (tb >= w0) & (tb <= w1)
        if m1.sum() < 50 or m2.sum() < 50:
            continue
        try:
            o, _ = cross_correlation_offset(ta[m1], ga[m1], tb[m2], gb[m2],
                                            max_offset_s, sample_dt)
            offs.append((w0, w1, o * 1000.0))
        except Exception:
            continue
    return offs


def estimate_sync_offset(cfg, A, B, dataA, dataB, t_hA, t_rA, t_hB, t_rB):
    """跨话题同步分析，返回结果 dict（含 mode 与各项 offset）。"""
    sync = cfg["sync"]
    thr = cfg["thresholds"]
    axis = sync["axis"]
    max_off = sync["max_offset_s"]
    dt = sync["sample_dt"]

    res = {
        "topicA": A, "topicB": B,
        "offset_candidate_ms": timestamp_coarse_offset(t_hA, t_rA, t_hB, t_rB) * 1000.0,
        "cross_out_of_order": cross_sensor_out_of_order(t_hA, t_rA, t_hB, t_rB),
        "anomaly": [],
    }

    if dataA is None or dataB is None:
        res["mode"] = "timestamp-only"
        return res

    ta, accA, gyroA = dataA
    tb, accB, gyroB = dataB
    ga = select_axis(gyroA, axis)
    gb = select_axis(gyroB, axis)

    # 按 header 时间排序后再互相关
    oa = np.argsort(ta)
    ob = np.argsort(tb)
    ta, ga = ta[oa], ga[oa]
    tb, gb = tb[ob], gb[ob]

    try:
        xcorr, curve = cross_correlation_offset(ta, ga, tb, gb, max_off, dt)
        res["mode"] = "cross-correlation"
        res["xcorr_offset_ms"] = xcorr * 1000.0
        res["corr_lags_ms"] = curve[0] * 1000.0
        res["corr_vals"] = curve[1]
        res["corr_peak"] = float(np.max(curve[1])) if len(curve[1]) else _NAN
    except Exception as e:
        res["mode"] = "timestamp-only"
        res["xcorr_error"] = str(e)
        return res

    try:
        med, mean, std, pairs = peak_matching_offset(ta, ga, tb, gb, dt)
        res["peak_offset_median_ms"] = med * 1000.0
        res["peak_offset_mean_ms"] = mean * 1000.0
        res["peak_offset_std_ms"] = std * 1000.0
        res["peak_matched_pairs"] = pairs
    except Exception as e:
        res["peak_error"] = str(e)

    res["sliding_windows"] = sliding_window_offsets(ta, ga, tb, gb, max_off, dt)

    # 综合采用的 offset：波峰匹配均值优先，否则互相关
    if "peak_offset_mean_ms" in res:
        chosen = res["peak_offset_mean_ms"]
    else:
        chosen = res["xcorr_offset_ms"]
    res["offset_ms"] = chosen

    if abs(chosen) > thr["offset_warn_ms"]:
        res["anomaly"].append(("时间同步偏移超限", float(ta[0]), chosen))

    return res


# ======================== 导出 ========================

def _fmt(v, fmt=".9f", empty_on_nan=True):
    if v is None:
        return "" if empty_on_nan else "nan"
    if isinstance(v, float) and np.isnan(v):
        return "" if empty_on_nan else "nan"
    return format(v, fmt)


def export_topic_csv(topic, t_h, t_r, diff_ms, res, out_dir):
    """导出单话题 CSV：t_h, t_r, diff_ms, delta_h, delta_r, is_out_of_order。"""
    path = out_dir / f"{topic}_timing_analysis.csv"
    n = len(t_h)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_h", "t_r", "diff_ms", "delta_h", "delta_r", "is_out_of_order"])
        for i in range(n):
            w.writerow([
                _fmt(t_h[i]),
                _fmt(t_r[i]),
                _fmt(diff_ms[i], ".6f"),
                _fmt(res["delta_h_row"][i]),
                _fmt(res["delta_r_row"][i]),
                "1" if res["ooo"][i] else "0",
            ])
    return path


def export_sync_csv(res, dataA, dataB, cfg, out_dir):
    """导出跨 IMU 对比 CSV：公共时间轴 + 两套所选角速度信号。"""
    if dataA is None or dataB is None or "corr_vals" not in res:
        return None
    axis = cfg["sync"]["axis"]
    dt = cfg["sync"]["sample_dt"]
    ta, accA, gyroA = dataA
    tb, accB, gyroB = dataB
    ga = select_axis(gyroA, axis)
    gb = select_axis(gyroB, axis)
    oa, ob = np.argsort(ta), np.argsort(tb)
    ta, ga = ta[oa], ga[oa]
    tb, gb = tb[ob], gb[ob]

    lo = max(ta[0], tb[0])
    hi = min(ta[-1], tb[-1])
    nt1, ng1 = _resample_uniform(ta, ga, dt)
    nt2, ng2 = _resample_uniform(tb, gb, dt)
    m1 = (nt1 >= lo) & (nt1 <= hi)
    m2 = (nt2 >= lo) & (nt2 <= hi)
    t1, s1 = nt1[m1], ng1[m1]
    t2, s2 = nt2[m2], ng2[m2]
    min_len = min(len(t1), len(t2))
    t1, s1, s2 = t1[:min_len], s1[:min_len], s2[:min_len]

    path = out_dir / f"{res['topicA']}_vs_{res['topicB']}_sync.csv"
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", f"gyro_{axis}_{res['topicA']}", f"gyro_{axis}_{res['topicB']}"])
        for i in range(min_len):
            w.writerow([_fmt(t1[i]), _fmt(s1[i], ".6f"), _fmt(s2[i], ".6f")])
    return path


# ======================== 报告 ========================

def _pct(v):
    return "nan" if (isinstance(v, float) and np.isnan(v)) else f"{v * 100:.2f}%"


def write_report(results, topic_order, cross, anomalies, cfg, out_dir, input_dir):
    """写 Markdown 汇总报告。"""
    lines = []
    add = lines.append

    add("# 多传感器时序 / 时间同步分析报告")
    add("")
    add(f"- 输入目录：`{input_dir}`")
    add(f"- 分析话题数：{len(topic_order)}")
    add("")
    add("## 限制说明（重要）")
    add("")
    add("- `t_r`（received_timestamp）是 **bag-record 进程** 的接收时间，不是业务算法节点的接收时间；")
    add("- 乱序、延迟尖峰、断流均为 **bag 录制视角**，只能作为实机风险提示，不等于算法节点表现；")
    add("- 丢帧率是基于 header 时间戳 + 理论频率的 **估计值**，不等于硬件真实丢包；")
    add("- 若无 IMU 信号，跨话题 offset 仅为时间戳层面比对，不是真实波形偏移。")
    add("")

    add("## 单话题汇总")
    add("")
    add("| topic | 帧数 | 周期ms | Δh均值 | Δhσ | Δh最大 | Δr均值 | Δr最大 | 乱序(占比) | "
        "diff均值 | diff p95 | diff max | 丢帧率 |")
    add("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
    for topic in topic_order:
        r = results[topic]
        if not r.get("valid"):
            add(f"| {topic} | 0 | - | - | - | - | - | - | - | - | - | - | - |")
            continue
        period_ms = r["period"] * 1000.0 if r["period"] else _NAN
        dh, dr, d = r["delta_h"], r["delta_r"], r["diff"]
        ooo = f"{r['out_of_order_count']} ({_pct(r['out_of_order_ratio'])})"
        add(f"| {topic} | {r['n']} | {_fmt(period_ms, '.2f')} | {_fmt(dh['mean']*1000, '.2f')} | "
            f"{_fmt(dh['std']*1000, '.2f')} | {_fmt(dh['max']*1000, '.2f')} | "
            f"{_fmt(dr['mean']*1000, '.2f')} | {_fmt(dr['max']*1000, '.2f')} | {ooo} | "
            f"{_fmt(d['mean'], '.2f')} | {_fmt(r['diff_p95'], '.2f')} | {_fmt(d['max'], '.2f')} | "
            f"{_pct(r['loss_rate'])} |")
    add("")

    if cross:
        add("## 跨话题时间同步")
        add("")
        add(f"- 对比对：`{cross['topicA']}` (基准) vs `{cross['topicB']}`")
        add(f"- 模式：{cross['mode']}")
        add(f"- 时间戳粗同步 offset_candidate（按接收时间对齐的 header 差中位数，含传输延迟差）"
            f" = {_fmt(cross['offset_candidate_ms'], '.3f')} ms")
        if cross["mode"] == "cross-correlation":
            add(f"- 互相关 offset：{_fmt(cross['xcorr_offset_ms'], '.3f')} ms "
                f"(互相关峰值 {_fmt(cross.get('corr_peak'), '.3f')})")
            if "peak_offset_mean_ms" in cross:
                add(f"- 波峰波谷匹配 offset：均值 {_fmt(cross['peak_offset_mean_ms'], '.3f')} ms，"
                    f"中位数 {_fmt(cross['peak_offset_median_ms'], '.3f')} ms，"
                    f"标准差 {_fmt(cross['peak_offset_std_ms'], '.3f')} ms，"
                    f"匹配对数 {cross['peak_matched_pairs']}")
            if "peak_error" in cross:
                add(f"- 波峰匹配失败：{cross['peak_error']}")
            add(f"- 采用 offset：{_fmt(cross['offset_ms'], '.3f')} ms "
                f"({'topic2 相对 topic1 滞后' if cross['offset_ms'] > 0 else 'topic2 相对 topic1 超前'})")
            if cross["sliding_windows"]:
                add("- 滑动窗口分段 offset（ms，观察漂移）：")
                for w0, w1, o in cross["sliding_windows"]:
                    add(f"  - [{_fmt(w0, '.2f')}, {_fmt(w1, '.2f')}] -> {_fmt(o, '.3f')} ms")
        elif "xcorr_error" in cross:
            add(f"- 互相关失败，退化为时间戳比对：{cross['xcorr_error']}")
        add(f"- 跨传感器到达乱序事件（{cross['topicA']} 采样更早但接收晚于 {cross['topicB']}）："
            f"{cross['cross_out_of_order']}")
        add("")
        add("> 正值表示 topic2 的 header 时间戳相对 topic1 滞后。")
        add("")

    if anomalies:
        add("## 异常事件列表")
        add("")
        add("| topic | 类型 | 发生时间(t_h) | 数值 |")
        add("| --- | --- | ---: | ---: |")
        for topic, typ, t, val in anomalies:
            add(f"| {topic} | {typ} | {_fmt(t)} | {_fmt(val, '.4f')} |")
        add("")

    add("## 解读提示")
    add("")
    add("- **驱动/传感器侧阻塞堆积**：`t_h` 挤在一起（Δh 出现大量爆帧）、`t_r` 仍拉开；")
    add("- **DDS/订阅侧队列积压**：`t_h` 间隔均匀、`t_r` 扎堆（Δr 出现接收断流后紧跟密集到达）；")
    add("- **延迟尖峰**：diff 短暂超过阈值，通常与系统调度抖动或队列积压有关；")
    add("- **diff 整体漂移**：暗示系统时间源（header 时钟 vs 主机时钟）发生偏移；")
    add("- **跨 IMU offset 漂移**：滑动窗口 offset 逐渐变化，说明两传感器时间基准在相对漂移。")
    add("")

    path = out_dir / "timing_report.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


# ======================== 可选绘图 ========================

def _setup_cjk_font():
    """配置 matplotlib 中文字体，避免子图标题中文显示为方框（缺字）。

    优先探测系统中文字体文件并显式注册，命中则返回字体名；找不到时回退到
    按已注册字体名匹配。同时关闭 unicode_minus，保证负号正常显示。
    """
    import matplotlib
    import matplotlib.font_manager as fm

    matplotlib.rcParams["axes.unicode_minus"] = False

    candidates = [
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",          # 文泉驿微米黑
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",  # Noto Sans CJK
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

    known = set(f.name for f in fm.fontManager.ttflist)
    for name in ("WenQuanYi Micro Hei", "Noto Sans CJK SC", "Noto Sans CJK JP",
                 "Droid Sans Fallback", "AR PL UMing CN", "SimHei", "Microsoft YaHei"):
        if name in known:
            matplotlib.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            return name
    return None


def _add_hstat_lines(ax, values):
    """在 ax 上叠加均值 / p95 / p99 三条水平参考线（自动忽略 NaN）。

    用于折线图：把数据整体水平（均值）与 95%、99% 分位数标出来，直观看出
    典型值 vs 极端尖峰。values 单位应与纵轴一致。
    """
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


def make_plots(results, topic_order, cross, cfg, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    _setup_cjk_font()

    plots_dir = out_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    plot_files = []  # (caption, png_path)

    # 单话题：diff / delta_h / delta_r
    for topic in topic_order:
        r = results[topic]
        if not r.get("valid"):
            continue
        t_h = r["t_h"]; t_r = r["t_r"]; diff = r["diff_ms"]
        dh_ms = r["delta_h_row"] * 1000
        dr_ms = r["delta_r_row"] * 1000
        fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=False)
        axs[0].plot(t_h, diff, "-", markersize=1)
        axs[0].set_ylabel("diff (ms)")
        axs[0].set_title(f"{topic} - diff (t_r - t_h)")
        _add_hstat_lines(axs[0], diff)
        axs[1].plot(t_h, dh_ms, "-", markersize=1)
        axs[1].set_ylabel("delta_h (ms)")
        axs[1].set_title(f"{topic} - 采样间隔 (按 t_h 排序)")
        _add_hstat_lines(axs[1], dh_ms)
        axs[2].plot(t_r, dr_ms, "-", markersize=1)
        axs[2].set_ylabel("delta_r (ms)")
        axs[2].set_xlabel("t_r (s)")
        axs[2].set_title(f"{topic} - 到达间隔")
        _add_hstat_lines(axs[2], dr_ms)
        for ax in axs:
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7)
        fig.tight_layout()
        png = plots_dir / f"{topic}_timing.png"
        fig.savefig(png, dpi=120)
        plt.close(fig)
        plot_files.append((f"{topic} 时序图（diff / 采样间隔 / 到达间隔）", str(png)))

    # 跨话题：互相关曲线
    if cross and cross["mode"] == "cross-correlation" and "corr_lags_ms" in cross:
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(cross["corr_lags_ms"], cross["corr_vals"])
        ax.axvline(x=cross["xcorr_offset_ms"], color="r", linestyle="--",
                   label=f"offset = {cross['xcorr_offset_ms']:.3f} ms")
        ax.set_xlabel("lag (ms)")
        ax.set_ylabel("归一化相关系数")
        ax.set_title(f"互相关曲线 {cross['topicA']} vs {cross['topicB']}")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        png = plots_dir / "cross_correlation.png"
        fig.savefig(png, dpi=120)
        plt.close(fig)
        plot_files.append((f"互相关曲线 {cross['topicA']} vs {cross['topicB']}", str(png)))

    print(f"图表已保存到 {plots_dir.resolve()}")
    return plot_files


def _register_pdf_cjk_font():
    """注册 reportlab 中文字体：优先同时覆盖拉丁+中文的字体，回退内置 CID 字体；返回字体名。

    注意：DroidSansFallbackFull 只有中文、没有 ASCII，用它会让标题里的字母/数字/斜杠
    全部变成方框，故不能选它；wqy-microhei（与 matplotlib 同一字体）同时覆盖两种。
    """
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.cidfonts import UnicodeCIDFont
    from reportlab.pdfbase.ttfonts import TTFont

    # 首选与 matplotlib 一致的文泉驿微米黑（拉丁 + 中文全覆盖）
    if Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc").exists():
        try:
            pdfmetrics.registerFont(TTFont("CJK", "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
                                           subfontIndex=0))
            return "CJK"
        except Exception:
            pass
    # 回退：内置 STSong-Light（Adobe-GB1 CID，同样覆盖拉丁 + 中文）
    pdfmetrics.registerFont(UnicodeCIDFont("STSong-Light"))
    return "STSong-Light"


def write_pdf(results, topic_order, cross, out_dir, input_dir, plot_files):
    """把统计表格 + 各 topic 图 + 互相关图组装成单个 PDF 报告。"""
    from reportlab.lib.pagesizes import A4
    from reportlab.lib import colors
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import mm
    from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table,
                                    TableStyle, Image, PageBreak)

    font_name = _register_pdf_cjk_font()

    path = out_dir / "timing_report.pdf"
    doc = SimpleDocTemplate(str(path), pagesize=A4,
                            leftMargin=14 * mm, rightMargin=14 * mm,
                            topMargin=14 * mm, bottomMargin=14 * mm,
                            title="多传感器时序分析报告")
    ss = getSampleStyleSheet()
    title_st = ParagraphStyle("T", parent=ss["Title"], fontName=font_name, fontSize=16)
    h2_st = ParagraphStyle("H2", parent=ss["Heading2"], fontName=font_name,
                           fontSize=12, spaceBefore=8, spaceAfter=4)
    body_st = ParagraphStyle("Body", parent=ss["Normal"], fontName=font_name,
                             fontSize=9, leading=13)
    cell_st = ParagraphStyle("Cell", parent=ss["Normal"], fontName=font_name,
                             fontSize=6.5, leading=8)
    head_st = ParagraphStyle("Head", parent=cell_st, fontName=font_name,
                             fontSize=7, leading=9)

    story = []
    story.append(Paragraph("多传感器时序 / 时间同步分析报告", title_st))
    story.append(Spacer(1, 3 * mm))
    story.append(Paragraph(f"输入目录：{input_dir}", body_st))
    story.append(Paragraph(f"分析话题数：{len(topic_order)}", body_st))
    story.append(Spacer(1, 2 * mm))

    # 一、单话题汇总表
    story.append(Paragraph("一、单话题汇总", h2_st))
    header = ["topic", "帧数", "周期ms", "Δh均值", "Δhσ", "Δh最大", "Δr均值",
              "Δr最大", "乱序(占比)", "diff均值", "diff p95", "diff max", "丢帧率"]
    data = [[Paragraph(h, head_st) for h in header]]
    for topic in topic_order:
        r = results[topic]
        if not r.get("valid"):
            row = [topic, "0", "-", "-", "-", "-", "-", "-", "-", "-", "-", "-", "-"]
        else:
            period_ms = r["period"] * 1000.0 if r["period"] else _NAN
            dh, dr, d = r["delta_h"], r["delta_r"], r["diff"]
            ooo = f"{r['out_of_order_count']} ({_pct(r['out_of_order_ratio'])})"
            row = [topic, str(r["n"]), _fmt(period_ms, ".2f"),
                   _fmt(dh["mean"] * 1000, ".2f"), _fmt(dh["std"] * 1000, ".2f"),
                   _fmt(dh["max"] * 1000, ".2f"), _fmt(dr["mean"] * 1000, ".2f"),
                   _fmt(dr["max"] * 1000, ".2f"), ooo,
                   _fmt(d["mean"], ".2f"), _fmt(r["diff_p95"], ".2f"),
                   _fmt(d["max"], ".2f"), _pct(r["loss_rate"])]
        data.append([Paragraph(str(c), cell_st) for c in row])
    col_widths = [30 * mm, 10 * mm, 11 * mm, 11 * mm, 10 * mm, 11 * mm, 11 * mm,
                  11 * mm, 16 * mm, 12 * mm, 12 * mm, 12 * mm, 12 * mm]
    tbl = Table(data, colWidths=col_widths, repeatRows=1)
    tbl.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.4, colors.grey),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e8eef7")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ALIGN", (0, 0), (0, -1), "LEFT"),
        ("ALIGN", (1, 0), (-1, -1), "RIGHT"),
    ]))
    story.append(tbl)
    story.append(Spacer(1, 4 * mm))

    # 二、跨话题时间同步
    if cross:
        story.append(Paragraph("二、跨话题时间同步", h2_st))
        lines = [
            f"对比对：{cross['topicA']} (基准) vs {cross['topicB']}",
            f"模式：{cross['mode']}",
            f"时间戳粗同步 offset_candidate（含传输延迟差）= {_fmt(cross['offset_candidate_ms'], '.3f')} ms",
        ]
        if cross["mode"] == "cross-correlation":
            lines.append(f"互相关 offset：{_fmt(cross['xcorr_offset_ms'], '.3f')} ms"
                         f"（峰值 {_fmt(cross.get('corr_peak'), '.3f')}）")
            if "peak_offset_mean_ms" in cross:
                lines.append(f"波峰波谷匹配 offset：均值 {_fmt(cross['peak_offset_mean_ms'], '.3f')} ms，"
                             f"中位数 {_fmt(cross['peak_offset_median_ms'], '.3f')} ms，"
                             f"标准差 {_fmt(cross['peak_offset_std_ms'], '.3f')} ms，"
                             f"匹配对数 {cross['peak_matched_pairs']}")
            lines.append(f"采用 offset：{_fmt(cross['offset_ms'], '.3f')} ms")
        lines.append(f"跨传感器到达乱序事件：{cross['cross_out_of_order']}")
        for ln in lines:
            story.append(Paragraph(ln, body_st))
        story.append(Spacer(1, 3 * mm))

    # 三、统计图（每张一页）
    if plot_files:
        story.append(Paragraph("三、统计图", h2_st))
        from PIL import Image as PILImage
        for caption, png in plot_files:
            if not Path(png).exists():
                continue
            with PILImage.open(png) as im:
                ratio = im.size[1] / im.size[0]
            width = 180 * mm
            height = min(width * ratio, 250 * mm)
            story.append(PageBreak())
            story.append(Paragraph(caption, h2_st))
            story.append(Spacer(1, 2 * mm))
            story.append(Image(png, width=width, height=height))

    doc.build(story)
    return path


# ======================== 主流程 ========================

def main():
    cfg = load_config()

    input_dir = cfg["input_dir"]
    if not input_dir:
        print("请先在 timing_analysis_config.yaml 中配置 input_dir。", file=sys.stderr)
        return

    topics = discover_topics(input_dir, cfg["topic_filter"])
    if not topics:
        print(f"在 {input_dir} 下未找到 *_timing.txt 文件。", file=sys.stderr)
        return
    print(f"发现 {len(topics)} 个 topic：")
    for t in sorted(topics):
        print(f"  {t}")

    out_dir = Path(cfg["output_dir"]) if cfg["output_dir"] else Path(input_dir) / "timing_report"
    out_dir.mkdir(parents=True, exist_ok=True)

    thr = cfg["thresholds"]

    results = {}       # topic -> 分析结果 dict（含逐帧数组）
    raw = {}           # topic -> (t_h, t_r, diff_ms)
    all_anomalies = []  # (topic, 类型, 时间, 数值)

    for topic in sorted(topics):
        parsed = parse_timing_file(topics[topic])
        if parsed is None:
            print(f"[跳过] {topic}: 无有效数据", file=sys.stderr)
            continue
        t_h, t_r, diff_ms, skipped = parsed
        if skipped:
            print(f"[提示] {topic}: 跳过 {skipped} 行表头/非法行", file=sys.stderr)
        period = theoretical_period(topic, cfg["sensor_freq_hz"])
        res, anom = analyze_single_topic(topic, t_h, t_r, diff_ms, period, thr)
        res["t_h"], res["t_r"], res["diff_ms"] = t_h, t_r, diff_ms
        results[topic] = res
        raw[topic] = (t_h, t_r, diff_ms)
        all_anomalies.extend([(topic, typ, t, v) for typ, t, v in anom])
        export_topic_csv(topic, t_h, t_r, diff_ms, res, out_dir)
        print(f"  {topic:40s} {res['n']:>8} 帧")

    # 跨话题同步
    cross = None
    sync_topics = cfg["sync"]["topics"]
    if len(sync_topics) == 2 and all(t in raw for t in sync_topics):
        A, B = sync_topics
        dataA = load_imu_signal(A, topics[A])
        dataB = load_imu_signal(B, topics[B])
        t_hA, t_rA, _ = raw[A]
        t_hB, t_rB, _ = raw[B]
        cross = estimate_sync_offset(cfg, A, B, dataA, dataB, t_hA, t_rA, t_hB, t_rB)
        if cross["anomaly"]:
            all_anomalies.extend([(f"{A} vs {B}", typ, t, v)
                                  for typ, t, v in cross["anomaly"]])
        export_sync_csv(cross, dataA, dataB, cfg, out_dir)
        off = cross.get("offset_ms")
        off_str = f"{off:.3f} ms" if off is not None else "nan"
        print(f"\n跨话题同步（{A} vs {B}）: mode={cross['mode']}, offset={off_str}")
    else:
        print(f"\n[提示] sync.topics={sync_topics} 未全部匹配到 *_timing.txt，跳过跨话题同步。",
              file=sys.stderr)

    report_path = write_report(results, sorted(results), cross, all_anomalies,
                               cfg, out_dir, input_dir)

    plot_files = []
    if cfg["plot"]["enabled"]:
        try:
            plot_files = make_plots(results, sorted(results), cross, cfg, out_dir)
        except Exception as e:
            print(f"[警告] 绘图失败：{e}", file=sys.stderr)

    # 生成 PDF 汇总报告（统计表格 + 各 topic 图 + 互相关图）
    try:
        pdf_path = write_pdf(results, sorted(results), cross,
                             out_dir, input_dir, plot_files)
        print(f"PDF 报告：{pdf_path.resolve()}")
    except Exception as e:
        print(f"[警告] 生成 PDF 失败：{e}", file=sys.stderr)

    print(f"\n分析完成，报告：{report_path.resolve()}")
    print(f"输出目录：{out_dir.resolve()}")


if __name__ == "__main__":
    main()
