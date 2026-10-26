import os
import numpy as np
import numpy.ma as npm
import matplotlib.pyplot as plt
import pandas as pd
import math
import json
import yaml
import glob
import logging
import utm
from scipy.spatial.transform import Rotation
from datetime import datetime, timezone
import geojson
from geojson import Feature, Point, FeatureCollection


logging.basicConfig(level=logging.INFO)



def mat2RPY(mat):
    roll = math.atan2(mat[2, 1], mat[2, 2])
    pitch = math.atan2(-mat[2, 0],
                       math.sqrt(mat[2, 1] ** 2 + mat[2, 2] ** 2))
    yaw = math.atan2(mat[1, 0], mat[0, 0])
    return np.array([roll, pitch, yaw])


def mat2YPR(mat):
    roll = math.atan2(-mat[1, 2], mat[2, 2])
    pitch = math.atan2(mat[0, 2], math.sqrt(
        mat[1, 2] * mat[1, 2] + mat[2, 2] * mat[2, 2]))
    yaw = math.atan2(-mat[0, 1], mat[0, 0])
    return np.array([roll, pitch, yaw])


def RPY2mat(roll, pitch, yaw):
    R = np.zeros((3, 3), dtype=np.float64)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    R[0][0] = cy * cp
    R[0][1] = cy * sp * sr - sy * cr
    R[0][2] = cy * sp * cr + sy * sr
    R[1][0] = sy * cp
    R[1][1] = sy * sp * sr + cy * cr
    R[1][2] = sy * sp * cr - cy * sr
    R[2][0] = -sp
    R[2][1] = cp * sr
    R[2][2] = cp * cr
    R = np.mat(R)

    return R


def YPR2mat(roll, pitch, yaw):
    R = np.zeros((3, 3), dtype=float)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    R[0][0] = cy * cp
    R[0][1] = -sy * cp
    R[0][2] = sp
    R[1][0] = cy * sp * sr + cr * sy
    R[1][1] = -sy * sr * sp + cy * cr
    R[1][2] = -cp * sr
    R[2][0] = -cy * sp * cr + sy * sr
    R[2][1] = sy * sp * cr + cy * sr
    R[2][2] = cp * cr

    return R


def getTvcs(roll, pitch, yaw, x, y, z, order='ypr'):
    rpy_mat = YPR2mat(roll, pitch, yaw)
    if order == 'rpy':
        rpy_mat = RPY2mat(roll, pitch, yaw)
    T = np.zeros((4, 4), dtype=np.float64)
    T[:3, :3] = rpy_mat
    T[0][3] = x
    T[1][3] = y
    T[2][3] = z
    T[3][3] = 1
    T = np.mat(T)
    # exit(0)
    return T


def getT(rpyxyz, order='ypr'):
    rpy_mat = YPR2mat(rpyxyz[0], rpyxyz[1], rpyxyz[2])
    if order == 'rpy':
        rpy_mat = RPY2mat(rpyxyz[0], rpyxyz[1], rpyxyz[2])
    T = np.zeros((4, 4), dtype=np.float64)
    T[:3, :3] = rpy_mat
    T[0][3] = rpyxyz[3]
    T[1][3] = rpyxyz[4]
    T[2][3] = rpyxyz[5]
    T[3][3] = 1
    T = np.mat(T)
    # exit(0)
    return T


def getTvcsaxis(camera_info):
    roll = camera_info["roll"]
    pitch = camera_info["pitch"]
    yaw = camera_info["yaw"]
    x = camera_info["camera_x"]
    y = camera_info["camera_y"]
    z = camera_info["camera_z"]
    rpy_vcs_local = camera_info["vcs"]["rotation"]
    xyz_vcs_local = camera_info["vcs"]["translation"]
    T_local_axis = getTvcs(roll, pitch, yaw, x, y, z)
    T_vcs_loal = getTvcs(rpy_vcs_local[0], rpy_vcs_local[1], rpy_vcs_local[2],
                         xyz_vcs_local[0], xyz_vcs_local[1], xyz_vcs_local[2])

    return T_vcs_loal * T_local_axis


def getTlidaraxis(camera_info):
    x = camera_info["extrinsic"]["translation0"]["x"]
    y = camera_info["extrinsic"]["translation0"]["y"]
    z = camera_info["extrinsic"]["translation0"]["z"]
    qx = camera_info["extrinsic"]["rotation0"]["q.x"]
    qy = camera_info["extrinsic"]["rotation0"]["q.y"]
    qz = camera_info["extrinsic"]["rotation0"]["q.z"]
    qw = camera_info["extrinsic"]["rotation0"]["q.w"]
    rot = Rotation.from_quat(np.array([qx, qy, qz, qw]))
    mat = rot.as_matrix()
    T_cam_lidar = np.eye(4, dtype=np.float64)
    T_cam_lidar[:3, :3] = mat
    T_cam_lidar[0][3] = x
    T_cam_lidar[1][3] = y
    T_cam_lidar[2][3] = z
    T_lidar_cam = np.linalg.inv(T_cam_lidar)
    T_cam_axis = np.mat([[0, -1, 0, 0],
                         [0, 0, -1, 0],
                         [1, 0, 0, 0],
                         [0, 0, 0, 1]])
    return T_lidar_cam * T_cam_axis


def getTcamlidar(camera_yaml_dict):
    x = camera_yaml_dict["extrinsic"]["translation0"]["x"]
    y = camera_yaml_dict["extrinsic"]["translation0"]["y"]
    z = camera_yaml_dict["extrinsic"]["translation0"]["z"]
    qx = camera_yaml_dict["extrinsic"]["rotation0"]["q.x"]
    qy = camera_yaml_dict["extrinsic"]["rotation0"]["q.y"]
    qz = camera_yaml_dict["extrinsic"]["rotation0"]["q.z"]
    qw = camera_yaml_dict["extrinsic"]["rotation0"]["q.w"]
    rot = Rotation.from_quat(np.array([qx, qy, qz, qw]))
    mat = rot.as_matrix()
    T_cam_lidar = np.eye(4, dtype=np.float64)
    T_cam_lidar[:3, :3] = mat
    T_cam_lidar[0][3] = x
    T_cam_lidar[1][3] = y
    T_cam_lidar[2][3] = z
    return T_cam_lidar


def T_cam_axis():
    return np.mat([[0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [1, 0, 0, 0],
                   [0, 0, 0, 1]])


def GetDict(file):
    with open(file, 'r') as fs:
        dic = yaml.load(fs, Loader=yaml.Loader)
        return dic


def DumpDict(file, dic):
    with open(file, 'w') as fs:
        yaml.dump(dic, fs)


def GetDictJson(file):
    with open(file, 'r') as fs:
        dic = json.load(fs)
        return dic


def DumpDictJson(file, dic):
    with open(file, 'w') as fs:
        json.dump(dic, fs, indent=2)


# 画出文件中列数据的曲线
def plot_cols(file_path, cols_list, tag_list, axs_list, use_scatter=False):
    df = pd.read_csv(file_path, delimiter=' ', header=None)
    for i in range(0, len(cols_list)):
        col_idx = cols_list[i]
        tag = tag_list[i]
        axs = axs_list[i]
        if use_scatter:
            axs.scatter(df[0], df[col_idx], label=tag, s=2, c='r')
        else:
            axs.plot(df[0], df[col_idx], marker='o', markersize=2, label=tag)
        axs.grid(True)
        axs.legend()


def plot_data_list(x_data, y_data_list, tag_list, axs_list):
    for i in range(0, len(y_data_list)):
        y_data = y_data_list[i]
        tag = tag_list[i]
        axs = axs_list[i]
        axs.plot(x_data, y_data, marker='o', markersize=2, label=tag)
        axs.grid(True)
        axs.legend()


def filter_files_by_name_glob(root_path, target_name, filter_tag='/'):
    # 构造匹配模式，递归搜索所有文件
    pattern = f"**/{target_name}"
    matching_files = glob.glob(os.path.join(root_path, pattern), recursive=True)
    filted_files = []
    for file_path in matching_files:
        # 检查文件名是否包含指定的过滤标签
        if filter_tag not in file_path:
            continue
        filted_files.append(file_path)
    filted_files.sort()
    return filted_files


def convert_timeseg_to_utc(time_str):
    # 解析字符串为 datetime 对象
    dt = datetime.strptime(time_str, "%Y%m%d-%H%M%S_%f")

    # 转换为毫秒级的时间戳
    timestamp_ms = int(dt.timestamp() * 1000)
    return timestamp_ms



def utc_timestamp_to_datetime_str(timestamp: float) -> str:
    """将UTC时间戳转换为年月日时分秒格式字符串
    
    Args:
        timestamp: UTC时间戳（秒）
    
    Returns:
        格式化的时间字符串，如 "2024-01-15 10:30:45"
    """
    dt = datetime.fromtimestamp(timestamp)
    return dt.strftime("%Y-%m-%d %H:%M:%S")




# 常量定义
PI = 3.14159265358979323846
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI
A_Earth = 6378137.0
WGS84_E2 = 0.00669437999014131699613723354004

def zoneNum(s_zone):
    if len(s_zone) != 3:
        return -1
    try:
        return int(s_zone[:2])
    except Exception:
        return -1

def UTMLetterDesignator(Lat):
    if (84 >= Lat) and (Lat >= 72):
        return 'X'
    elif (72 > Lat) and (Lat >= 64):
        return 'W'
    elif (64 > Lat) and (Lat >= 56):
        return 'V'
    elif (56 > Lat) and (Lat >= 48):
        return 'U'
    elif (48 > Lat) and (Lat >= 40):
        return 'T'
    elif (40 > Lat) and (Lat >= 32):
        return 'S'
    elif (32 > Lat) and (Lat >= 24):
        return 'R'
    elif (24 > Lat) and (Lat >= 16):
        return 'Q'
    elif (16 > Lat) and (Lat >= 8):
        return 'P'
    elif (8 > Lat) and (Lat >= 0):
        return 'N'
    elif (0 > Lat) and (Lat >= -8):
        return 'M'
    elif (-8 > Lat) and (Lat >= -16):
        return 'L'
    elif (-16 > Lat) and (Lat >= -24):
        return 'K'
    elif (-24 > Lat) and (Lat >= -32):
        return 'J'
    elif (-32 > Lat) and (Lat >= -40):
        return 'H'
    elif (-40 > Lat) and (Lat >= -48):
        return 'G'
    elif (-48 > Lat) and (Lat >= -56):
        return 'F'
    elif (-56 > Lat) and (Lat >= -64):
        return 'E'
    elif (-64 > Lat) and (Lat >= -72):
        return 'D'
    elif (-72 > Lat) and (Lat >= -80):
        return 'C'
    else:
        return 'Z'

def setRMNGconvert(latituderad, height):
    sl2 = math.sin(latituderad) ** 2
    sq = math.sqrt(1.0 - WGS84_E2 * sl2)
    R_m = A_Earth * (1.0 - WGS84_E2) / (sq ** 3)
    R_n = A_Earth / sq

    g0 = 9.7803267714
    a1 = 0.0053022
    a2 = 0.0000058
    a3 = 3.0877
    a4 = 0.0044
    a5 = 0.072

    s22 = math.sin(2.0 * latituderad) ** 2
    s2 = math.sin(latituderad) ** 2
    s4 = s2 * s2
    r_l = g0 * (1.0 + a1 * s2 - a2 * s22)
    grav = r_l - (a3 - a4 * s2) * (height / 1000000.0) + a5 * ((height / 1000000.0) ** 2)
    return R_m, R_n, grav

def LLtoUTM(Lat, Long):
    a = 6378137.0
    eccSquared = WGS84_E2
    k0 = 0.9996

    LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180
    LatRad = Lat * DEG_TO_RAD
    LongRad = LongTemp * DEG_TO_RAD

    ZoneNumber = int((LongTemp + 180) / 6) + 1

    if Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0:
        ZoneNumber = 32

    if Lat >= 72.0 and Lat < 84.0:
        if LongTemp >= 0.0 and LongTemp < 9.0:
            ZoneNumber = 31
        elif LongTemp >= 9.0 and LongTemp < 21.0:
            ZoneNumber = 33
        elif LongTemp >= 21.0 and LongTemp < 33.0:
            ZoneNumber = 35
        elif LongTemp >= 33.0 and LongTemp < 42.0:
            ZoneNumber = 37

    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3
    LongOriginRad = LongOrigin * DEG_TO_RAD

    UTMZone = f"{ZoneNumber:02d}{UTMLetterDesignator(Lat)}"

    eccPrimeSquared = eccSquared / (1 - eccSquared)

    N = a / math.sqrt(1 - eccSquared * math.sin(LatRad) ** 2)
    T = math.tan(LatRad) ** 2
    C = eccPrimeSquared * math.cos(LatRad) ** 2
    A = math.cos(LatRad) * (LongRad - LongOriginRad)

    M = a * ((1 - eccSquared / 4 - 3 * eccSquared ** 2 / 64 - 5 * eccSquared ** 3 / 256) * LatRad
             - (3 * eccSquared / 8 + 3 * eccSquared ** 2 / 32 + 45 * eccSquared ** 3 / 1024) * math.sin(2 * LatRad)
             + (15 * eccSquared ** 2 / 256 + 45 * eccSquared ** 3 / 1024) * math.sin(4 * LatRad)
             - (35 * eccSquared ** 3 / 3072) * math.sin(6 * LatRad))

    UTMEasting = (k0 * N * (A + (1 - T + C) * A ** 3 / 6
                            + (5 - 18 * T + T ** 2 + 72 * C - 58 * eccPrimeSquared) * A ** 5 / 120)
                  + 500000.0)

    UTMNorthing = (k0 * (M + N * math.tan(LatRad) * (A ** 2 / 2
                                                     + (5 - T + 9 * C + 4 * C ** 2) * A ** 4 / 24
                                                     + (61 - 58 * T + T ** 2 + 600 * C - 330 * eccPrimeSquared) * A ** 6 / 720)))
    if Lat < 0:
        UTMNorthing += 10000000.0

    return UTMNorthing, UTMEasting, UTMZone

def LLtoUTMFixZone(Lat, Long, ZoneNumber):
    if Lat > 90.0 or Lat < -90.0 or Long > 180.0 or Long < -180.0:
        return Lat, Long
    a = 6378137.0
    eccSquared = WGS84_E2
    k0 = 0.9996

    LongTemp = (Long + 180.0) - int((Long + 180.0) / 360.0) * 360 - 180.0
    LatRad = Lat * DEG_TO_RAD
    LongRad = LongTemp * DEG_TO_RAD

    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3
    LongOriginRad = LongOrigin * DEG_TO_RAD

    eccPrimeSquared = eccSquared / (1.0 - eccSquared)

    N = a / math.sqrt(1.0 - eccSquared * math.sin(LatRad) ** 2)
    T = math.tan(LatRad) ** 2
    C = eccPrimeSquared * math.cos(LatRad) ** 2
    A = math.cos(LatRad) * (LongRad - LongOriginRad)
    M = a * ((1.0 - eccSquared / 4.0 - 3.0 * eccSquared ** 2 / 64.0 - 5.0 * eccSquared ** 3 / 256.0) * LatRad
             - (3.0 * eccSquared / 8.0 + 3.0 * eccSquared ** 2 / 32.0 + 45.0 * eccSquared ** 3 / 1024.0) * math.sin(2.0 * LatRad)
             + (15.0 * eccSquared ** 2 / 256.0 + 45.0 * eccSquared ** 3 / 1024.0) * math.sin(4.0 * LatRad)
             - (35.0 * eccSquared ** 3 / 3072.0) * math.sin(6.0 * LatRad))

    UTMEasting = (k0 * N * (A + (1.0 - T + C) * A ** 3 / 6.0
                            + (5.0 - 18.0 * T + T ** 2 + 72.0 * C - 58.0 * eccPrimeSquared) * A ** 5 / 120.0)
                  + 500000.0)

    UTMNorthing = (k0 * (M + N * math.tan(LatRad) * (A ** 2 / 2.0
                                                     + (5.0 - T + 9.0 * C + 4.0 * C ** 2) * A ** 4 / 24.0
                                                     + (61.0 - 58.0 * T + T ** 2 + 600.0 * C - 330.0 * eccPrimeSquared) * A ** 6 / 720.0)))
    if Lat < 0.0:
        UTMNorthing += 10000000.0

    return UTMNorthing, UTMEasting

def ENUYawToUTMYaw(wgs84_a_lat, wgs84_a_lon, enu_yaw, zone_number):
    # 输入经纬度为度，enu_yaw为弧度
    Rm, Rn, g = setRMNGconvert(wgs84_a_lat * DEG_TO_RAD, 0.0)
    de = 1000.0 * math.cos(enu_yaw)
    dn = 1000.0 * math.sin(enu_yaw)
    lfbegin_lat = wgs84_a_lat
    lfbegin_lon = wgs84_a_lon
    lfend_lat = lfbegin_lat + (dn / Rm) * RAD_TO_DEG
    lfend_lon = lfbegin_lon + (de / (Rn * math.cos(wgs84_a_lat * DEG_TO_RAD))) * RAD_TO_DEG

    utm_northing_a, utm_easting_a = LLtoUTMFixZone(lfbegin_lat, lfbegin_lon, zone_number)
    utm_northing_b, utm_easting_b = LLtoUTMFixZone(lfend_lat, lfend_lon, zone_number)

    delt_utm_x = utm_easting_b - utm_easting_a
    delt_utm_y = utm_northing_b - utm_northing_a

    utm_yaw = math.atan2(delt_utm_y, delt_utm_x)
    return utm_yaw




def draw_arrow(axs, x, y, angle, length, width, color='blue'):
    """
    在指定位置画箭头。
    
    参数:
    - x, y: 箭头起点的位置 (x, y)。
    - angle: 箭头的角度（以弧度为单位，0表示水平向右）。
    - length: 箭头的长度。
    - width: 箭头的宽度。
    - color: 箭头的颜色。
    """
    # 计算箭头的终点位置
    dx = length * np.cos(angle)  # x方向的变化
    dy = length * np.sin(angle)  # y方向的变化

    # 使用 plt.arrow 绘制箭头
    axs.arrow(x, y, dx, dy, head_width=width, head_length=width*2, fc=color, ec=color)


def read_csv_as_list(file_path, delimiter):
    data_list = []
    if not os.path.exists(file_path):
        print('Not find:', file_path)
        return data_list
    # 判断文件是否为空
    if os.path.getsize(file_path) == 0:
        print('File is empty:', file_path)
        return data_list
    df = pd.read_csv(file_path, delimiter=delimiter, header=None)
    for index, row in df.iterrows():
        data_list.append(row.tolist())
    return data_list



# 模块级变量，记录固定的 UTM Zone
_fixed_utm_zone_number = None
_fixed_utm_zone_letter = None

def latlon_to_utm(latitude, longitude, force_zone_number=None):
    """
    将经纬度转换为 UTM 坐标。
    
    Args:
        latitude: 纬度（度）
        longitude: 经度（度）
        force_zone_number: 强制使用的 Zone 号（可选），None 表示自动确定
        
    Returns:
        (easting, northing, zone_number, zone_letter)
    """
    global _fixed_utm_zone_number, _fixed_utm_zone_letter
    
    if force_zone_number is not None:
        # 使用指定的 Zone
        utm_coords = utm.from_latlon(latitude, longitude, force_zone_number=force_zone_number)
    elif _fixed_utm_zone_number is not None:
        # 使用已固定的 Zone
        utm_coords = utm.from_latlon(latitude, longitude, force_zone_number=_fixed_utm_zone_number)
    else:
        # 首次调用，自动确定 Zone 并记录
        utm_coords = utm.from_latlon(latitude, longitude)
        _fixed_utm_zone_number = utm_coords[2]
        _fixed_utm_zone_letter = utm_coords[3]
        logging.info(f"UTM Zone fixed to: {_fixed_utm_zone_number}{_fixed_utm_zone_letter}")
    
    return utm_coords

def reset_utm_zone():
    """重置固定的 UTM Zone，下次调用 latlon_to_utm 时重新自动确定"""
    global _fixed_utm_zone_number, _fixed_utm_zone_letter
    _fixed_utm_zone_number = None
    _fixed_utm_zone_letter = None
    logging.info("UTM Zone reset")


def set_utm_zone(zone_number, zone_letter=None):
    """手动设置固定的 UTM Zone"""
    global _fixed_utm_zone_number, _fixed_utm_zone_letter
    _fixed_utm_zone_number = zone_number
    _fixed_utm_zone_letter = zone_letter
    logging.info(f"UTM Zone manually set to: {zone_number}{zone_letter or ''}")

def convert_llh_file_to_tum(input_llh_file, output_tum_file, roll_col_idx=-1, pitch_col_idx=-1, yaw_col_idx=3):
    if not os.path.exists(input_llh_file):
        logging.warning("LLH file not found!")
        return
    llh_data = read_csv_as_list(input_llh_file, ' ')
    with open(output_tum_file, 'w') as f:
        for each_data in llh_data:
            # 假设每行数据格式为: timestamp lat lon alt
            if len(each_data) < 3:
                continue
            timestamp = each_data[0]
            lat = float(each_data[1])
            lon = float(each_data[2])
            height = float(each_data[3]) if len(each_data) > 3 else 0.0
            enu_yaw = float(each_data[-1])  # 默认最后一列为yaw
            roll = 0.0
            pitch = 0.0
            if len(each_data) > yaw_col_idx:
                enu_yaw = float(each_data[yaw_col_idx])
            if len(each_data) > pitch_col_idx and pitch_col_idx > -1:
                pitch = float(each_data[pitch_col_idx])
            if len(each_data) > roll_col_idx and roll_col_idx > -1:
                roll = float(each_data[roll_col_idx])
            # 将LLH转换为TUM格式
            # 使用固定 Zone 的 latlon_to_utm
            utm_easting, utm_northing, zone_number, zone_letter = latlon_to_utm(lat, lon)
            utm_yaw = ENUYawToUTMYaw(lat, lon, enu_yaw, zone_number)
            rot = RPY2mat(roll, pitch, utm_yaw)
            quat = Rotation.from_matrix(rot).as_quat()
            # TUM格式: timestamp tx ty tz qx qy qz qw
            tum_line = f"{timestamp} {utm_easting:.6f} {utm_northing:.6f} {height:.6f} {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n"
            f.write(tum_line)
    logging.info("Dumped TUM file: {}".format(output_tum_file))


def combine_pos_hdg_to_tum(pos_file, hdg_file, tum_file, max_dt=0.5):
    """把 GNSS 位置(llh)与航向(yaw/pitch)按时间戳匹配，输出 UTM 坐标系下的 TUM 轨迹。

    pos_file: 位置文件，每行 "timestamp lat lon alt ..."（lat/lon 单位度，alt 单位米）。
    hdg_file: 航向文件，每行 "timestamp hdg_type baseline heading pitch ..."
              （heading/pitch 单位度）。GnssHdgMsg 的 heading 定义为
              "counterclockwise from east [deg]"，即 ENU 系 yaw；pitch 向上为正。
    tum_file: 输出 TUM 文件，每行 "timestamp easting northing height qx qy qz qw"。
              时间戳取 pos 的；heading/pitch 取 hdg 中时间最近的一条（|dt|<=max_dt）。
    max_dt: 匹配允许的最大时间差（秒）。

    返回写入的帧数。
    """
    from bisect import bisect_left

    if not os.path.exists(pos_file) or not os.path.exists(hdg_file):
        logging.warning("pos/hdg file not found!")
        return 0

    pos_data = []  # (ts_float, ts_str, lat, lon, alt)
    with open(pos_file, 'r') as f:
        for line in f:
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                ts = float(parts[0])
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3])
            except ValueError:
                continue  # 表头行
            pos_data.append((ts, parts[0], lat, lon, alt))

    hdg_data = []  # (ts_float, heading_deg, pitch_deg)
    with open(hdg_file, 'r') as f:
        for line in f:
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                ts = float(parts[0])
                heading = float(parts[3])   # GnssHdgMsg: heading(deg)
                pitch = float(parts[4])     # GnssHdgMsg: pitch(deg)
            except ValueError:
                continue
            hdg_data.append((ts, heading, pitch))

    if not pos_data or not hdg_data:
        logging.warning("pos/hdg file has no valid data!")
        return 0

    hdg_ts = [h[0] for h in hdg_data]

    reset_utm_zone()  # 首条数据固定 UTM zone
    n = 0
    with open(tum_file, 'w') as f:
        for ts, ts_str, lat, lon, alt in pos_data:
            # 二分查找最近时间戳的航向
            i = bisect_left(hdg_ts, ts)
            best_j = -1
            best_dt = None
            for j in (i - 1, i):
                if 0 <= j < len(hdg_data):
                    dt = abs(hdg_data[j][0] - ts)
                    if dt <= max_dt and (best_dt is None or dt < best_dt):
                        best_dt = dt
                        best_j = j
            if best_j < 0:
                continue

            heading_rad = hdg_data[best_j][1] * DEG_TO_RAD  # 度 -> 弧度
            pitch_rad = hdg_data[best_j][2] * DEG_TO_RAD
            roll = 0.0

            easting, northing, zone_number, zone_letter = latlon_to_utm(lat, lon)
            utm_yaw = ENUYawToUTMYaw(lat, lon, heading_rad, zone_number)
            rot = RPY2mat(roll, pitch_rad, utm_yaw)
            quat = Rotation.from_matrix(rot).as_quat()  # [x, y, z, w]

            f.write(f"{ts_str} {easting:.6f} {northing:.6f} {alt:.6f} "
                    f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n")
            n += 1
    logging.info("Dumped TUM file: {}".format(tum_file))
    return n


def dump_rawgnss_geojson(data, file, ts_interval=0.01):
    if not data:
        print('No valid data to dump:', file)
        return
    last_ts = -1.0
    with open(file, 'w') as f:
        feature_list = []
        for each_data in data:
            ts = float(each_data[0])
            if last_ts > 0 and (ts - last_ts) < ts_interval:
                continue
            last_ts = ts
            lat = float(each_data[1])
            lon = float(each_data[2])
            info = 0
            if len(each_data) > 3:
                info = each_data[-1]
            my_feature = Feature(geometry=Point((lon, lat)),
                                 properties={"time": each_data[0],
                                             "info": info})
            feature_list.append(my_feature)
        geojson.dump(FeatureCollection(feature_list), f)


def create_geojson(llh_file, ts_interval=0.01):
    if not os.path.exists(llh_file):
        logging.warning("LLH file not found!")
        return
    output_geojson_file = llh_file.replace('.txt', '.geojson')
    llh_data = read_csv_as_list(llh_file, ' ')
    dump_rawgnss_geojson(llh_data, output_geojson_file, ts_interval)
    logging.info("Dumped geojson file: {}".format(output_geojson_file))
