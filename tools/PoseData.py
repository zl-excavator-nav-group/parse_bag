import os
import re
import utils
import copy
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation


# 定义位姿结构体
class Pose:
    def __init__(self, timestamp, xyz, quat):
        self.timestamp = timestamp
        self.T = np.eye(4, dtype=np.float64)
        rot = Rotation.from_quat(np.array(quat)).as_matrix()
        self.T[:3, :3] = rot
        dien = np.transpose(rot) * rot
        dien = np.linalg.inv(rot) * rot
        dien = Rotation.from_matrix(rot).as_matrix()
        quat = Rotation.from_matrix(dien).as_quat()
        self.T[0][3] = xyz[0]
        self.T[1][3] = xyz[1]
        self.T[2][3] = xyz[2]
        self.eular = utils.mat2RPY(rot)  # 实际的循序是z-y-x
    def get_tum_data(self):
        data = []
        data.append(self.timestamp)
        data.append(self.T[0][3])
        data.append(self.T[1][3])
        data.append(self.T[2][3])
        quat = Rotation.from_matrix(self.T[:3, :3]).as_quat()
        data.append(quat[0])
        data.append(quat[1])
        data.append(quat[2])
        data.append(quat[3])
        return data

# 计算车体坐标系的平移误差向量与角度误差， 从pos2指向pos1
def cal_pose_err(pos1, pos2):
    trans_err_vec = pos1.T[:3, 3:] - pos2.T[:3, 3:]
    # 将两个轨迹点平移量的偏差转到本提坐标系
    trans_err_vec = np.dot(np.transpose(pos2.T[:3, :3]), trans_err_vec)
    rpy_err_vec = pos1.eular - pos2.eular
    return trans_err_vec, rpy_err_vec



# 从轨迹文件中加载得到轨迹列表
def read_trajectory_file(file_path):
    pose_dq = deque()
    with open(file_path, 'r') as file:
        for line in file:
            data = list(map(float, line.strip().split()))
            pose = Pose(data[0], [data[1], data[2], data[3]],
                        [data[4], data[5], data[6], data[7]])
            pose_dq.append(pose)
    return pose_dq

def output_tum_file(all_pos_dq, file_path):
    if len(all_pos_dq) == 0:
        return
    with open(file_path, 'w') as file:
        for pos in all_pos_dq:
            tum_data = pos.get_tum_data()
            for j in range(0, len(tum_data)):
                if j == (len(tum_data) - 1):
                    file.write(str(tum_data[j]))
                else:
                    file.write(str(tum_data[j]) + ' ')
            file.write('\n')

# 从位姿队列中获取时间戳及坐标list
def get_coords(pose_dq):
    all_tp = []
    x_coords = []
    y_coords = []
    z_coords = []
    all_yaw = []
    if (len(pose_dq) == 0):
        return
    for pos in pose_dq:
        all_tp.append(pos.timestamp)
        x_coords.append(pos.T[0, 3])
        y_coords.append(pos.T[1, 3])
        z_coords.append(pos.T[2, 3])
        all_yaw.append(pos.eular[2])
    return all_tp, x_coords, y_coords, z_coords, all_yaw

# 根据坐标列表计算总里程
def cal_dis(odo_xs, odo_ys):
    dis = 0
    for i in range(1, len(odo_xs)):
        dis += np.sqrt(pow(odo_xs[i] - odo_xs[i - 1], 2) +
                       pow(odo_ys[i] - odo_ys[i - 1], 2))
    return dis

# 计算位姿队列的总长度
def cal_traj_length(pose_dq):
    if len(pose_dq) == 0:
        return 0
    all_tp, x_coords, y_coords, z_coords, _ = get_coords(pose_dq)
    traj_length = cal_dis(x_coords, y_coords)
    return traj_length

def cal_traj_delta_dis(pose_dq, start_tp=0, end_tp=0):
    e_idx = 1
    cur_seg_dis = 0
    while e_idx < len(pose_dq):
        if start_tp > 0 and pose_dq[e_idx].timestamp < start_tp:
            e_idx += 1
            continue
        trans_err, _ = cal_pose_err(
            pose_dq[e_idx - 1], pose_dq[e_idx])
        cur_seg_dis += np.linalg.norm(trans_err)
        e_idx += 1
        if end_tp > 0 and pose_dq[e_idx].timestamp > end_tp:
            break
    return cur_seg_dis

# 从pack解出来的info文件中加载
def read_caninfo_file(file_path, use_stand_data = True):
    all_speed = deque()
    all_yawrate = deque()
    all_tp = deque()
    all_wa = deque()
    all_gear = deque()
    with open(file_path, 'r') as file:
        for line in file:
            data = list(map(float, line.strip().split()))
            if use_stand_data or abs(data[1]) > 0:
                all_tp.append(data[0])
                all_speed.append(data[1])
                all_yawrate.append(data[3])
                all_wa.append(data[4])
                all_gear.append(data[5])
    return all_tp, all_speed, all_yawrate, all_wa, all_gear


def read_colums_from_file(file_path, cols):
    if len(cols) == 0:
        return
    cols_data = [[] for _ in range(len(cols))]
    with open(file_path, 'r') as file:
        for line in file:
            data = list(map(float, line.strip().split()))
            for i in range(len(cols)):
                cols_data[i].append(data[cols[i]])
    return cols_data


def split_file_by_tp(input_path, output_path,
                     start_timestamp, end_timestamp):
    if os.path.exists(output_path):
        os.remove(output_path)
    with open(input_path, 'r') as file, open(output_path, 'w') as output_file:
        for line in file:
            timestamp = float(line.split()[0])  # 假设时间戳在每行数据的第一列
            if timestamp < start_timestamp:
                continue
            if timestamp > end_timestamp:
                break
            output_file.write(line)


def get_int_from_string(string):
    match = re.search(r'\d+', string)
    number = 1e5
    if match:
        number = int(match.group())
    return number


# 从两个位姿队列中找到时间戳最近的匹配对
def get_sync_idxs(odo_pose_dq, gnss_pose_dq, sync_time=20 * 1e-3):
    sync_idxs = []  # 匹配对list
    odo_idx = 0
    gnss_idx = 0
    # 根据gnss的时间戳，找最近的odo的时间戳
    while gnss_idx < len(gnss_pose_dq):
        while odo_idx < len(odo_pose_dq) - 1 and (
                odo_pose_dq[odo_idx].timestamp < gnss_pose_dq[gnss_idx].timestamp):
            odo_idx += 1
        pre_odo_idx = max(0, odo_idx - 1)
        # 选择时间戳最近的一个
        cur_time_diff = abs(odo_pose_dq[odo_idx].timestamp - gnss_pose_dq[gnss_idx].timestamp)
        pre_time_diff = abs(odo_pose_dq[pre_odo_idx].timestamp - gnss_pose_dq[gnss_idx].timestamp)
        if pre_time_diff < cur_time_diff:
            odo_idx = pre_odo_idx
        if abs(odo_pose_dq[odo_idx].timestamp -
               gnss_pose_dq[gnss_idx].timestamp) < sync_time:
            sync_idxs.append((odo_idx, gnss_idx))
        gnss_idx += 1
        if gnss_idx % 100 == 0:
            progress = (gnss_idx) / len(gnss_pose_dq)
            print('get_sync_idxs progress: {:.2f}%'.format(progress*100))
    return sync_idxs


# 对齐两个位姿, roll,pitch不变，仍为绝对值；其他变为相对值
def adj_pose(T_start, T_end):
    T_start_cp = copy.deepcopy(T_start)
    start_rpy = utils.mat2RPY(T_start_cp[:3, :3])
    T_start_cp[:3, :3] = utils.RPY2mat(0, 0, start_rpy[2])
    T_end_adj = np.dot(np.linalg.inv(T_start_cp), T_end)
    return T_end_adj

# 计算位姿队列中指定索引范围内的高程增量
def cal_height_delta(pose_dq, start_idx, end_idx):
    if start_idx < 0 or end_idx >= len(pose_dq) or start_idx >= end_idx:
        return 0
    height_delta = 0
    # 均匀采样20个点计算高程增量
    sample_num = 20
    step = (end_idx - start_idx) // sample_num
    if step < 1:
        step = 1
    prev_z = pose_dq[start_idx].T[2, 3]
    for idx in range(start_idx + step, end_idx + 1, step):
        cur_z = pose_dq[idx].T[2, 3]
        height_delta += abs(cur_z - prev_z)
        prev_z = cur_z
    return height_delta
