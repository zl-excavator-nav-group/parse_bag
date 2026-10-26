import os
import logging
import utils
import PoseData
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from matplotlib.ticker import MultipleLocator

# 从tum文件中计算速度和角速度, 并利用位置差分计算航向角
def cal_speedgyro_from_tum(tum_path, cal_interval=1):
    output_path = os.path.dirname(tum_path)
    output_file_path = os.path.join(output_path, 'speed_gyro.txt')
    cal_yaw_pose_path = os.path.join(output_path, 'cal_yaw_pose.txt')
    all_pose_dq = PoseData.read_trajectory_file(tum_path)
    speed_list = []
    gyro_list = []
    ts_list = []
    cur_yaw = 0
    with open(output_file_path, 'w') as file, open(cal_yaw_pose_path, 'w') as file_yaw:
        for i in range(cal_interval, len(all_pose_dq)):
            dt = all_pose_dq[i].timestamp - all_pose_dq[i - cal_interval].timestamp
            delta_p = all_pose_dq[i].T[:3, 3] - all_pose_dq[i - cal_interval].T[:3, 3]
            if np.linalg.norm(delta_p) > 1e-3:
                cur_yaw = np.arctan2(delta_p[1], delta_p[0])
            speed = np.linalg.norm(delta_p) / dt
            cur_rpy = all_pose_dq[i].eular
            last_rpy = all_pose_dq[i - cal_interval].eular
            delta_rpy = cur_rpy - last_rpy
            for j in range(3):
                if delta_rpy[j] > np.pi:
                    delta_rpy[j] -= 2 * np.pi
                if delta_rpy[j] < -np.pi:
                    delta_rpy[j] += 2 * np.pi
            gyro = delta_rpy / dt
            speed_list.append(speed)
            gyro_list.append(gyro)
            ts_list.append(all_pose_dq[i].timestamp)
            file.write(str(all_pose_dq[i].timestamp) + ' ' + str(speed) + ' ' +
                       str(gyro[0]) + ' ' + str(gyro[1]) + ' ' + str(gyro[2]) + ' ' + str(cur_rpy[2]) + '\n')
            R_cal = utils.RPY2mat(0, 0, cur_yaw)
            q_cal = Rotation.from_matrix(R_cal).as_quat()
            file_yaw.write(str(all_pose_dq[i].timestamp) + ' ' + str(all_pose_dq[i].T[0, 3]) + ' ' +
                           str(all_pose_dq[i].T[1, 3]) + ' ' + str(all_pose_dq[i].T[2, 3]) + ' ' +
                           str(q_cal[0]) + ' ' + str(q_cal[1]) + ' ' + str(q_cal[2]) + ' ' + str(q_cal[3]) + '\n')
    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax[0].plot(ts_list, speed_list)
    ax[0].set_ylabel('speed')
    ax[0].grid(True)
    ax[1].plot(ts_list, [gyro[2] for gyro in gyro_list])
    ax[1].set_ylabel('yawrate')
    ax[1].grid(True)
    fig.savefig(os.path.join(output_path, 'speed_gyro.png'), dpi=500)
    return speed_list, gyro_list, output_file_path


# 检查INS原始yaw角与通过位置差分计算的yaw角的差异
def check_ins_yaw(raw_ins_tum_path):
    cur_dir = os.path.dirname(raw_ins_tum_path)
    cal_yaw_pose_path = os.path.join(cur_dir, 'cal_yaw_pose.txt')
    speed_info_path = os.path.join(cur_dir, 'speed_gyro.txt')
    if not os.path.exists(raw_ins_tum_path) or not os.path.exists(cal_yaw_pose_path) \
          or not os.path.exists(speed_info_path):
        logging.error('file not exist!!!')
        return
    raw_gnss_pose_dq = PoseData.read_trajectory_file(raw_ins_tum_path)
    cal_yaw_pose_dq = PoseData.read_trajectory_file(cal_yaw_pose_path)
    speedinfo_list = PoseData.read_colums_from_file(speed_info_path, [0,1,4])
    info_ts_list = speedinfo_list[0]
    speed_list = speedinfo_list[1]
    yawrate_list = speedinfo_list[2]
    if len(cal_yaw_pose_dq) != len(speedinfo_list[0]):
        logging.error('data length not equal!!!')
        return
    while len(raw_gnss_pose_dq) > 0 and raw_gnss_pose_dq[0].timestamp < cal_yaw_pose_dq[0].timestamp:
        raw_gnss_pose_dq.popleft()
    if len(raw_gnss_pose_dq) < 50:
        logging.error('gnss pose too less!!!')
        return
    if len(raw_gnss_pose_dq) != len(cal_yaw_pose_dq):
        logging.error('data length not equal!!!')
        return
    
    yaw_diff_list = []
    raw_yaw_list = []
    for i in range(0, len(raw_gnss_pose_dq)):
        # 筛选条件: 速度大于0.1m/s 且 yawrate小于0.05rad/s
        if speed_list[i] < 0.1 or abs(yawrate_list[i]) > 0.05:
            continue
        raw_yaw = raw_gnss_pose_dq[i].eular[2]
        cal_yaw = cal_yaw_pose_dq[i].eular[2]
        yaw_diff = cal_yaw - raw_yaw
        if yaw_diff > np.pi:
            yaw_diff -= 2 * np.pi
        if yaw_diff < -np.pi:
            yaw_diff += 2 * np.pi
        cur_ts = raw_gnss_pose_dq[i].timestamp
        # 可以在这里限制时间
        # if cur_ts <= 1790040436.00:
        #     continue
        # if cur_ts >= 1790040477.00:
        #     break
        yaw_diff_list.append((cur_ts, yaw_diff))
        raw_yaw_list.append((cur_ts, raw_yaw))
    
    if len(yaw_diff_list) < 10:
        logging.error('yaw diff too less!!! speed too slow!!!')
        return

    mean_yaw_diff = np.mean([yaw[1] for yaw in yaw_diff_list])
    mean_diff_deg = mean_yaw_diff * 57.3
    logging.info('mean yaw diff:', mean_yaw_diff)
    logging.info('mean yaw diff deg:', mean_diff_deg)

    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    ax[0].yaxis.set_minor_locator(MultipleLocator(0.5))
    ax[0].plot([yaw[0] for yaw in yaw_diff_list], [yaw[1] * 57.3 for yaw in yaw_diff_list]) 
    ax[0].set_ylabel('yaw_diff')
    ax[0].axhline(y=mean_diff_deg, color='r', linestyle='--', label='mean yaw diff')
    ax[0].text(yaw_diff_list[0][0], mean_diff_deg, f'Mean yaw diff:{mean_diff_deg:.2f}deg', color='r', fontsize=10, va='bottom')
    ax[0].legend()
    ax[0].grid(True)

    ax[1].plot(info_ts_list, speed_list, label='speed')
    ax[1].set_ylabel('speed')
    ax[1].grid(True)

    ax[2].plot([yaw[0] for yaw in raw_yaw_list], [yaw[1] * 57.3 for yaw in raw_yaw_list])
    ax[2].set_ylabel('raw_yaw')
    ax[2].grid(True)
    fig.savefig(os.path.join(cur_dir, 'yaw_diff.png'), dpi=500)
    plt.show()
    return mean_diff_deg


if __name__ == '__main__':
    # 传入INS轨迹的tum文件路径
    tum_path = '/home/chenchen/code/parse_bag/output/2t/0922/rosbag2_2026_09_22-09_26_35_parsed_res/gps_pos_hdg_utm.txt'
    cal_speedgyro_from_tum(tum_path, 100)
    deg = check_ins_yaw(tum_path)
    print(deg)

