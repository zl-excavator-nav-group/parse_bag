# 对比组合定位输出的航向角 与 回转编码器输出的旋转角度
#
# 流程：
#   1. 从 gps_pos_hdg_utm.txt（TUM 格式：timestamp x y z qx qy qz qw）读取位姿，
#      用最后四列四元数解算原始 yaw 航向角；
#   2. 从 excavator_swing_encoder_state.txt 读取指定列的回转角度；
#   3. 两者分别做角度解缠（unwrap），再各自减去各自起点值，得到相对角度；
#   4. 将两条相对角度曲线画在同一个图上。
#
# 运行：python3 compare_yaw_swing_encoder.py

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ======================== 配置（按需修改） ========================
# 组合定位位姿轨迹文件（TUM 格式）
GPS_FILE = "/home/chenchen/code/parse_bag/output_0827/rosbag2_2026_08_27-08_53_39_parsed_res/gnss_odom.txt"

# 回转编码器状态文件
ENCODER_FILE = "/home/chenchen/code/parse_bag/output_0827/rosbag2_2026_08_27-08_53_39_parsed_res/excavator_swing_encoder_state.txt"

# 编码器文件中角度所在列（0 起始索引，对应 SwingEncoderState 消息展平后的列）。
# 该文件每行 22 列（第 0 列为 parse_bag 附加的 header 时间戳），其中第 9 / 10 / 11 列：
#   9  -> encoder_position_deg         （相对回转角，范围约 -143 ~ +216，未回绕）
#   10 -> encoder_position_wrapped_deg （= col9 回绕到 [-180, 180]）
#   11 -> encoder_position_rad         （字段名是弧度，实际存的是 [0, 360) 度 = col9 mod 360）
ENCODER_ANGLE_COL = 9

# 若两条曲线方向相反（两套数据的正方向约定不同），把下面改为 -1
ENCODER_SIGN = -1

# 横轴是否使用各自相对时间（True：都以各自起点为 0 秒；False：使用绝对时间戳）
USE_RELATIVE_TIME = True

# 输出图片路径（None 表示不保存，只弹出窗口显示）
OUTPUT_PNG = "/home/chenchen/code/parse_bag/output/compare_yaw_swing_encoder.png"
# ================================================================


def quat_to_yaw(qx, qy, qz, qw):
    """四元数 -> 航向角 yaw，单位度，范围 [-180, 180]。"""
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy ** 2 + qz ** 2))
    return np.rad2deg(yaw)


def unwrap_deg(angle_deg):
    """对角度序列（单位度）做解缠，得到连续的角度值。"""
    return np.rad2deg(np.unwrap(np.deg2rad(angle_deg)))


def main():
    # ---- 1. 读取组合定位轨迹，解算原始 yaw 角 ----
    gps_df = pd.read_csv(GPS_FILE, sep=r"\s+", header=None)
    gps_df.dropna(inplace=True)
    gps_ts = gps_df[0].to_numpy()
    # TUM 格式：0=timestamp, 1=x, 2=y, 3=z, 4=qx, 5=qy, 6=qz, 7=qw
    qx, qy, qz, qw = (gps_df[c].to_numpy() for c in range(4, 8))
    yaw_raw = quat_to_yaw(qx, qy, qz, qw)

    # ---- 2. 读取编码器回转角度 ----
    enc_df = pd.read_csv(ENCODER_FILE, sep=r"\s+", header=None)
    enc_df.dropna(inplace=True)
    enc_ts = enc_df[0].to_numpy()
    enc_ang_raw = enc_df[ENCODER_ANGLE_COL].to_numpy() * ENCODER_SIGN

    # ---- 3. 各自解缠后减去各自起点，得到相对角度 ----
    yaw_rel = unwrap_deg(yaw_raw)
    yaw_rel -= yaw_rel[0]

    enc_rel = unwrap_deg(enc_ang_raw)
    enc_rel -= enc_rel[0]

    # ---- 4. 横轴时间 ----
    if USE_RELATIVE_TIME:
        gps_x = gps_ts - gps_ts[0]
        enc_x = enc_ts - enc_ts[0]
        xlabel = "time since start (s)"
    else:
        gps_x = gps_ts
        enc_x = enc_ts
        xlabel = "timestamp (s)"

    # ---- 打印摘要，便于核对 ----
    print("===== yaw vs swing encoder =====")
    print(f"gps 点数: {len(gps_ts)}, 时间范围: {gps_ts[0]:.3f} ~ {gps_ts[-1]:.3f}")
    print(f"  原始 yaw 范围: [{yaw_raw.min():.3f}, {yaw_raw.max():.3f}] deg")
    print(f"  相对 yaw 范围: [{yaw_rel.min():.3f}, {yaw_rel.max():.3f}] deg")
    print(f"编码器点数: {len(enc_ts)}, 时间范围: {enc_ts[0]:.3f} ~ {enc_ts[-1]:.3f}")
    print(f"  原始角度范围 (col {ENCODER_ANGLE_COL}): "
          f"[{enc_ang_raw.min():.3f}, {enc_ang_raw.max():.3f}] deg")
    print(f"  相对角度范围: [{enc_rel.min():.3f}, {enc_rel.max():.3f}] deg")
    print("=================================")

    # ---- 5. 画在同一张图上 ----
    plt.figure(figsize=(10, 6))
    plt.plot(gps_x, yaw_rel, label="heading yaw (gps/ins)", linewidth=1.2)
    plt.plot(enc_x, enc_rel, label="swing encoder", linewidth=1.2)
    plt.xlabel(xlabel)
    plt.ylabel("relative angle (deg)")
    plt.title("Heading yaw vs swing encoder (relative to start)")
    plt.legend()
    plt.grid(True)

    if OUTPUT_PNG:
        plt.savefig(OUTPUT_PNG, dpi=150, bbox_inches="tight")
        print(f"图片已保存: {OUTPUT_PNG}")
    plt.show()


if __name__ == "__main__":
    main()
