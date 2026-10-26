#!/usr/bin/env python3
"""录制 ROS2 指定 topic 到 MCAP 或 SQLite3 bag。

配置（目录、topic 列表等）从 YAML 文件读取，见 record_config.yaml。
本质是对 `ros2 bag record` 的封装：读取配置 -> 拼命令 -> 前台运行，
并把 Ctrl+C (SIGINT) / SIGTERM 转发给子进程，保证 bag 正常落盘。

运行方式：
    python3 record_bag.py                 # 使用脚本同目录下的 record_config.yaml
    python3 record_bag.py --config xxx.yaml
"""

import argparse
import os
import re
import signal
import subprocess
import sys
from datetime import datetime
from typing import List, Optional

import yaml

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "record_config.yaml")


def load_config(config_path: str) -> dict:
    if not os.path.isfile(config_path):
        sys.exit(f"[错误] 配置文件不存在: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not cfg:
        sys.exit(f"[错误] 配置文件为空: {config_path}")
    return cfg


def validate_config(cfg: dict) -> None:
    record_dir = cfg.get("record_dir")
    if not record_dir:
        sys.exit("[错误] 配置项 record_dir（录制目录）不能为空")

    topics = cfg.get("topics") or []
    if not isinstance(topics, list) or not topics:
        sys.exit("[错误] 配置项 topics（待录制 topic 列表）不能为空，至少填写一个")

    storage = cfg.get("storage", "mcap")
    if storage not in ("mcap", "sqlite3"):
        sys.exit(f"[错误] 不支持的 storage: {storage}（可选 mcap / sqlite3）")

    storage_preset_profile = cfg.get("storage_preset_profile", "") or ""
    if storage_preset_profile not in ("", "none", "fastwrite", "zstd_fast", "zstd_small"):
        sys.exit(f"[错误] 不支持的 storage_preset_profile: {storage_preset_profile}"
                 "（可选空 / none / fastwrite / zstd_fast / zstd_small）")


def build_command(cfg: dict, output_dir: str) -> List[str]:
    topics = [str(t) for t in cfg["topics"]]
    storage = cfg.get("storage", "mcap")

    cmd = ["ros2", "bag", "record", "-s", storage, "-o", output_dir]

    max_bag_size = int(cfg.get("max_bag_size", 0) or 0)
    if max_bag_size > 0:
        cmd += ["--max-bag-size", str(max_bag_size)]

    max_cache = int(cfg.get("max_cache_size", 0) or 0)
    if max_cache > 0:
        cmd += ["--max-cache-size", str(max_cache)]

    # mcap 只支持自带的分块压缩（storage config / preset），不支持 ros2 通用 --compression-mode。
    storage_config_file = cfg.get("storage_config_file", "") or ""
    if storage_config_file:
        cmd += ["--storage-config-file", storage_config_file]
    elif storage == "mcap":
        storage_preset_profile = cfg.get("storage_preset_profile", "") or ""
        if storage_preset_profile:
            cmd += ["--storage-preset-profile", storage_preset_profile]

    regex = cfg.get("regex", "") or ""
    if regex:
        cmd += ["-e", regex]

    exclude = cfg.get("exclude", "") or ""
    if exclude:
        cmd += ["-x", exclude]

    cmd += topics
    return cmd


def check_storage_available(storage: str) -> None:
    """用实际执行的 ROS2 CLI 检查插件，避免 Python 环境与 CLI 环境不一致。"""
    try:
        result = subprocess.run(
            ["ros2", "bag", "record", "--help"],
            capture_output=True, text=True, timeout=15,
        )
    except FileNotFoundError:
        sys.exit("[错误] 找不到 ros2 命令，请先 source ROS2 和工作空间的 setup.bash")
    except (OSError, subprocess.TimeoutExpired) as exc:
        sys.exit(f"[错误] 无法检查 ROS2 存储插件: {exc}")
    if result.returncode:
        sys.exit(f"[错误] 无法读取 ros2 bag record 帮助:\n{result.stderr or result.stdout}")

    match = re.search(r"(?:-s|--storage)\s+\{([^}]+)\}", result.stdout)
    if not match:
        print("[警告] 无法从 ROS2 帮助中识别存储插件，将由录制命令检查")
        return
    available = [name.strip() for name in match.group(1).split(",")]
    if storage not in available:
        message = (f"[错误] 当前 ROS2 环境不支持 storage={storage}，"
                   f"可用插件: {', '.join(available)}")
        if storage == "mcap":
            message += (
                "\n请在录制机器上安装对应 ROS2 发行版的 MCAP 插件（apt 安装的 ROS2）："
                '\n  sudo apt update'
                '\n  sudo apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap'
                '\n然后重新 source /opt/ros/${ROS_DISTRO}/setup.bash 和工作空间的 setup.bash。'
                '\n或将录制配置改为 storage: "sqlite3"，脚本会跳过 MCAP 压缩预设。'
            )
        sys.exit(message)


def resolve_output_dir(record_dir: str) -> str:
    """在基础目录下生成一个全新的时间戳子目录作为本次 bag 输出路径。

    ros2 bag record -o <dir> 会把 <dir> 当作最终输出目录，且已存在会报错，
    因此每次录制都生成一个尚不存在的新目录，避免重复运行冲突。
    """
    out = os.path.join(record_dir, "rosbag2_" + datetime.now().strftime("%Y_%m_%d-%H_%M_%S"))
    candidate = out
    i = 1
    while os.path.exists(candidate):  # 同一秒内重复运行：追加 _1、_2 … 后缀
        candidate = f"{out}_{i}"
        i += 1
    return candidate


def run_record(cmd: List[str]) -> int:
    print("[录制] 执行命令:", " ".join(cmd))
    print("[录制] 按 Ctrl+C 停止录制并保存 bag ...")

    # 子进程放到独立进程组，便于整组转发信号。
    proc = subprocess.Popen(cmd, start_new_session=True)

    def forward_signal(signum, _frame):
        # 首次收到信号转发给子进程；后续（例如连按两次 Ctrl+C）强制退出。
        try:
            os.killpg(os.getpgid(proc.pid), signum)
        except ProcessLookupError:
            pass

    old_int = signal.signal(signal.SIGINT, forward_signal)
    old_term = signal.signal(signal.SIGTERM, forward_signal)

    try:
        return proc.wait()
    finally:
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="录制 ROS2 指定 topic 到 MCAP 或 SQLite3 bag")
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG,
        help=f"配置文件路径（默认: {DEFAULT_CONFIG}）",
    )
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    validate_config(cfg)

    print(f"[录制] 配置来源: {args.config}")
    storage = cfg.get("storage", "mcap")
    check_storage_available(storage)
    print(f"[录制] 存储格式: {storage}")
    if storage != "mcap" and cfg.get("storage_preset_profile"):
        print("[录制] 已跳过 MCAP 专用 storage_preset_profile（当前使用 sqlite3）")
    print(f"[录制] 基础目录: {cfg['record_dir']}")
    print(f"[录制] 待录制 topics ({len(cfg['topics'])}):")
    for t in cfg["topics"]:
        print(f"         - {t}")

    os.makedirs(cfg["record_dir"], exist_ok=True)
    output_dir = resolve_output_dir(cfg["record_dir"])
    print(f"[录制] 本次 bag 输出目录: {output_dir}")

    cmd = build_command(cfg, output_dir)
    return run_record(cmd)


if __name__ == "__main__":
    sys.exit(main())
