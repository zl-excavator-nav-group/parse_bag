# 用于对比不同IMU的数据

import math
import os
import sys
sys.path.append("..")
import re
import utils
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import PoseData


def compare_yawrate():
    is_angle_data = False
    all_file = ['/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_boche/ASM330#_20240628-114117_846_2_imu.txt',
                '/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_boche/Tele15#_20240628-114117_846_6_imu.txt',
                '/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_boche/INS570D#_20240628-114117_846_7_imu.txt']
    yawrate_data_cols = [[0, 3], [0, 3], [0, 3]]
    all_time_list = []
    all_yawrate_list = []
    all_yawrate_deg_list = []
    for i, data_file in enumerate(all_file):
        data_list = PoseData.read_colums_from_file(data_file, yawrate_data_cols[i])
        all_time_list.append(data_list[0])
        all_yawrate_list.append(data_list[1])
        yawrate_deg_list = [x *1 for x in data_list[1]]
        all_yawrate_deg_list.append(yawrate_deg_list)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(all_time_list[0], all_yawrate_deg_list[0], label='A_sample')
    ax.plot(all_time_list[1], all_yawrate_deg_list[1], label='on_rack')
    ax.plot(all_time_list[2], all_yawrate_deg_list[2], label='ins')
    ax.legend()
    plt.show()

def compare_imu_data():
    # all_file = ['/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_IMU_fan_on/bump/imu/20240701-162134_322/ASM330#_20240701-162134_322_2_imu.txt',
    #             '/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_IMU_fan_on/bump/Tele15#_20240701-162134_322_6_imu.txt']
    # all_file = ['/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_IMU_fan_on/high_way/imu/20240701-163500_324/ASM330#_20240701-163500_324_2_imu.txt',
    #             '/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_IMU_fan_on/high_way/Tele15#_20240701-163500_324_6_imu.txt']
    # all_file = ['/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_IMU_fan_on/ramps/imu/20240701-163345_992/ASM330#_20240701-163345_992_2_imu.txt',
    #             '/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_IMU_fan_on/ramps/Tele15#_20240701-163345_992_6_imu.txt']
    # all_file = ['/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_IMU_fan_off/static/imu/20240702-164107_803/ASM330#_20240702-164107_803_2_imu.txt',
    #             '/media/chenchenwei/199/ODO_DATASET/car3_imu_check/car3_IMU_fan_off/static/Tele15#_20240702-164107_803_6_imu.txt']
    all_file = ['/home/chenchen/code/parse_bag/output/rosbag2_2026_08_20-09_02_24_parsed_res/gps_imu_raw_base_link.txt',
                '/home/chenchen/code/parse_bag/output/rosbag2_2026_08_20-09_02_24_parsed_res/livox_imu_192_168_1_158_base_link.txt']
    file_tag = ['gps_imu_raw', 'livox_imu_192_168_1_158']
    # 加载所有的df
    df_list = []
    for file_path in all_file:
        df = pd.read_csv(file_path, delimiter=' ', header=None)
        df.dropna(inplace=True)
        # timestamp_threshold = 1706306163.000  # 1秒钟的阈值，单位微秒
        # df = df[df[0] < timestamp_threshold]
        df['ts_diff'] = df[0].diff()
        df['acc_x_diff'] = df[1].diff()
        df['acc_y_diff'] = df[2].diff()
        df['acc_z_diff'] = df[3].diff()
        df['gyro_x_diff'] = df[4].diff()
        df['gyro_y_diff'] = df[5].diff()
        df['gyro_z_diff'] = df[6].diff()
        df_list.append(df)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title('timestamp diff')
    for i, df in enumerate(df_list):
        tag = file_tag[i]
        ax.plot(df[0], df['ts_diff'], label=tag)
    ax.legend()


    fig, axs = plt.subplots(3, 1, sharex=True)
    fig.suptitle('acc_data', fontsize=15)
    for i, df in enumerate(df_list):
        # if i == 0:
        #     df[1] = [x*-1 for x in df[1]]
        #     df[2] = [x*-1 for x in df[2]]
        tag = file_tag[i]
        axs[0].plot(df[0], df[1], label=tag)
        axs[0].set_ylabel('acc_x')
        axs[1].plot(df[0], df[2], label=tag)
        axs[1].set_ylabel('acc_y')
        axs[2].plot(df[0], df[3], label=tag)
        axs[2].set_ylabel('acc_z')
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[0].grid(True)
    axs[1].grid(True)
    axs[2].grid(True)

    fig, axs = plt.subplots(3, 1, sharex=True)
    fig.suptitle('gyro_data', fontsize=15)
    for i, df in enumerate(df_list):
        # if i == 0:
        #     df[4] = [x*-1 for x in df[4]]
        #     df[5] = [x*-1 for x in df[5]]
        tag = file_tag[i]
        axs[0].plot(df[0], df[4], label=tag)
        axs[0].set_ylabel('gyro_x')
        axs[1].plot(df[0], df[5], label=tag)
        axs[1].set_ylabel('gyro_y')
        axs[2].plot(df[0], df[6], label=tag)
        axs[2].set_ylabel('gyro_z')
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[0].grid(True)
    axs[1].grid(True)
    axs[2].grid(True)

    fig, axs = plt.subplots(3, 1, sharex=True)
    fig.suptitle('acc_diff_data', fontsize=15)
    for i, df in enumerate(df_list):
        tag = file_tag[i]
        axs[0].plot(df[0], df['acc_x_diff'], label=tag)
        axs[0].set_ylabel('acc_x_diff')
        axs[1].plot(df[0], df['acc_y_diff'], label=tag)
        axs[1].set_ylabel('acc_y_diff')
        axs[2].plot(df[0], df['acc_z_diff'], label=tag)
        axs[2].set_ylabel('acc_z_diff')
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[0].grid(True)
    axs[1].grid(True)
    axs[2].grid(True)

    fig, axs = plt.subplots(3, 1, sharex=True)
    fig.suptitle('gyro_diff_data', fontsize=15)
    for i, df in enumerate(df_list):
        tag = file_tag[i]
        axs[0].plot(df[0], df['gyro_x_diff'], label=tag)
        axs[0].set_ylabel('gyro_x_diff')
        axs[1].plot(df[0], df['gyro_y_diff'], label=tag)
        axs[1].set_ylabel('gyro_y_diff')
        axs[2].plot(df[0], df['gyro_z_diff'], label=tag)
        axs[2].set_ylabel('gyro_z_diff')
    axs[0].legend()
    axs[1].legend()
    axs[2].legend()
    axs[0].grid(True)
    axs[1].grid(True)
    axs[2].grid(True)
    plt.show()

def integrate_angular(file_path, data_colums, bias, start_tp=0, end_tp=0):
    integ_ang = 0
    data_list = PoseData.read_colums_from_file(file_path, data_colums)
    for i in range(1, len(data_list[0])):
        if start_tp != 0 and data_list[0][i] < start_tp:
            continue
        if end_tp != 0 and data_list[0][i] > end_tp:
            break
        dt = data_list[0][i] - data_list[0][i-1]
        integ_ang += (data_list[1][i] - bias) * dt
    print('integ_ang_rad:', integ_ang)
    print('integ_ang_deg:', math.degrees(integ_ang))
    return integ_ang, math.degrees(integ_ang)



def calc_imu_statistics(file_path: str, start_tp: float = 0, end_tp: float = 0):
    """
    从 IMU 文本文件读取数据，计算指定时间区间内 acc 和 gyro 的均值与标准差。

    文件格式（空格分隔）：
        timestamp acc_x acc_y acc_z gyro_x gyro_y gyro_z

    Args:
        file_path: IMU 数据文件路径
        start_tp: 起始时间戳（秒），0 表示不限制
        end_tp: 结束时间戳（秒），0 表示不限制

    Returns:
        dict: 包含均值和标准差的字典
    """
    df = pd.read_csv(file_path, delimiter=' ', header=None)
    df.dropna(inplace=True)

    # 列索引：0=timestamp, 1=acc_x, 2=acc_y, 3=acc_z, 4=gyro_x, 5=gyro_y, 6=gyro_z
    col_names = ['timestamp', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z']
    df.columns = col_names[:len(df.columns)]

    # 时间区间过滤
    if start_tp > 0:
        df = df[df['timestamp'] >= start_tp]
    if end_tp > 0:
        df = df[df['timestamp'] <= end_tp]

    if len(df) == 0:
        print(f"警告：指定时间区间内无数据！start_tp={start_tp}, end_tp={end_tp}")
        return None

    # 计算均值
    acc_mean = df[['acc_x', 'acc_y', 'acc_z']].mean().to_numpy()
    gyro_mean = df[['gyro_x', 'gyro_y', 'gyro_z']].mean().to_numpy()

    # 计算标准差
    acc_std = df[['acc_x', 'acc_y', 'acc_z']].std().to_numpy()
    gyro_std = df[['gyro_x', 'gyro_y', 'gyro_z']].std().to_numpy()
    gyro_std_norm = np.linalg.norm(gyro_std)

    # 计算最大值
    acc_max = df[['acc_x', 'acc_y', 'acc_z']].max().to_numpy()
    gyro_max = df[['gyro_x', 'gyro_y', 'gyro_z']].max().to_numpy()

    # 计算最小值
    acc_min = df[['acc_x', 'acc_y', 'acc_z']].min().to_numpy()
    gyro_min = df[['gyro_x', 'gyro_y', 'gyro_z']].min().to_numpy()

    # 计算峰峰值（最大值 - 最小值）
    acc_peak_to_peak = acc_max - acc_min
    gyro_peak_to_peak = gyro_max - gyro_min

    result = {
        'acc_mean': acc_mean,
        'acc_std': acc_std,
        'acc_max': acc_max,
        'acc_min': acc_min,
        'acc_peak_to_peak': acc_peak_to_peak,
        'gyro_mean': gyro_mean,
        'gyro_std': gyro_std,
        'gyro_std_norm': gyro_std_norm,
        'gyro_max': gyro_max,
        'gyro_min': gyro_min,
        'gyro_peak_to_peak': gyro_peak_to_peak,
        'sample_count': len(df),
        'time_range': (df['timestamp'].iloc[0], df['timestamp'].iloc[-1])
    }

    # 打印结果
    print(f"===== IMU 统计结果 =====")
    print(f"文件: {file_path}")
    print(f"样本数: {result['sample_count']}")
    print(f"时间范围: {result['time_range'][0]:.3f} ~ {result['time_range'][1]:.3f}")
    print(f"--- Acc (x, y, z) ---")
    print(f"  均值: {acc_mean[0]:.6f}, {acc_mean[1]:.6f}, {acc_mean[2]:.6f}")
    print(f"  标准差: {acc_std[0]:.6f}, {acc_std[1]:.6f}, {acc_std[2]:.6f}")
    print(f"  最大值: {acc_max[0]:.6f}, {acc_max[1]:.6f}, {acc_max[2]:.6f}")
    print(f"  最小值: {acc_min[0]:.6f}, {acc_min[1]:.6f}, {acc_min[2]:.6f}")
    print(f"  峰峰值: {acc_peak_to_peak[0]:.6f}, {acc_peak_to_peak[1]:.6f}, {acc_peak_to_peak[2]:.6f}")
    print(f"--- Gyro (x, y, z) ---")
    print(f"  均值: {gyro_mean[0]:.6f}, {gyro_mean[1]:.6f}, {gyro_mean[2]:.6f}")
    print(f"  标准差: {gyro_std[0]:.6f}, {gyro_std[1]:.6f}, {gyro_std[2]:.6f}")
    print(f"  最大值: {gyro_max[0]:.6f}, {gyro_max[1]:.6f}, {gyro_max[2]:.6f}")
    print(f"  最小值: {gyro_min[0]:.6f}, {gyro_min[1]:.6f}, {gyro_min[2]:.6f}")
    print(f"  峰峰值: {gyro_peak_to_peak[0]:.6f}, {gyro_peak_to_peak[1]:.6f}, {gyro_peak_to_peak[2]:.6f}")
    print(f"  标准差范数: {gyro_std_norm:.6f}")
    print(f"========================")

    return result

if __name__ == '__main__':
    compare_imu_data()


    # file_path = '/home/chenchenwei/code/odom_raw_slot_odo_optimize/odometry/holm_egomotion/scripts/tools/data_analysis/imu_data.txt'
    # start_tp = 1717603213.601
    # end_tp = start_tp + 15
    # calc_imu_statistics(file_path, start_tp, end_tp)

    # 积分指定列的角速度数据
    # caninfo_3d = '/media/chenchenwei/199/ODO_DATASET/3dodo/3dodo_dataset/urban_highway/XP14_EG5796/20240901/imu/20240901-051309_969/SMI240#_20240901-051309_969_87_imu.txt'
    # # caninfo_3d = '/media/chenchenwei/199/ODO_DATASET/3dodo/3dodo_dataset/urban_highway/XP14_EG5796/20240902/2dodo_newgyrobias/20240902-125329_186/caninfo.txt'
    # yawrate_data_cols = [0, 4]
    # start_tp = 1725140028.839
    # end_tp = 1725140041.20
    # dt = end_tp - start_tp
    # bias = -0.00024
    # # bias = 0.00731
    # # bias = 0.00903
    # print('dt:', dt)
    # integrate_angular(caninfo_3d, yawrate_data_cols, bias, start_tp, end_tp)