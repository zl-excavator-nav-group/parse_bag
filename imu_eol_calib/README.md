# IMU 静态标定（EOL）工具

Python 复刻 C++ 出厂静态标定算法，输入 `parse_bag.py` 解析出的 IMU 原始数据文本文件，
输出与 C++ `DataAdapter::SaveImuEolRes` 同构的标定参数 JSON。

## 输入格式

`gps_imu_raw.txt` 之类，空格分隔、无表头、7 列：

```
timestamp acc_x acc_y acc_z gyro_x gyro_y gyro_z
```

- `timestamp`：秒（小数部分为纳秒），如 `1787218156.872075822`
- `acc_*`：m/s²
- `gyro_*`：rad/s

## 算法

逐行移植自 `holm_egomotion/src/calibration/factory/imu/vcs_imu_fac.cpp::VcsImuFacCalib::CalibProcess`：

1. 在指定时间范围内统计 acc / gyro 均值与标准差（样本标准差 n−1）；
2. 由 acc 均值（重力方向）用 Gram-Schmidt 解算 roll / pitch；yaw 由配置中的 `qCI` 强制覆盖；
3. `gyro_bias = mean(gyro)`；`acc_bias = mean(acc) − R_CIᵀ·[0,0,9.81]ᵀ`；
4. roll / pitch 超差检查（相对 `standard_roll/standard_pitch`，阈值 `rpy_thred`，单位度）；
5. 可选的 acc/gyro/丢帧/时间戳质量检查（`enable_check`，默认关闭，与 C++ 一致）。

## 快速开始

### 1. 先扫描静止窗口

整段数据通常不是全程静止的，先用滑窗扫出最静止的一段：

```bash
python3 imu_eol_calib/imu_static_calib.py imu_eol_calib/config_example.yaml --scan
```

输出按 `|gyro_std|` 升序排列的候选窗口，末尾直接给出可粘贴的 `start_timestamp / end_timestamp`。

### 2. 执行标定

把候选窗口填进配置（或直接用命令行覆盖）：

```bash
python3 imu_eol_calib/imu_static_calib.py imu_eol_calib/config_example.yaml \
    --start 1787218230 --end 1787218245.8
```

通过检查后，在 `file_directory` 下生成三个文件：

- `imu_calibration.json` — 完整标定参数（euler/四元数/tci/acc_bias/gyro_bias）
- `imu_calibration_checksum.json` — CRC16/CCITT-FALSE 校验
- `imu_param.json` — 精简参数（type + euler_angle_ci_z + tci）

### 3. 命令行参数

| 参数 | 说明 |
|---|---|
| `config` | YAML 配置文件路径（必填） |
| `--imu-file` | 覆盖配置中的 `imu_file` |
| `--start` / `--end` | 覆盖 `start_timestamp` / `end_timestamp`（绝对 epoch 秒，或 <1e6 时按相对首帧秒偏移） |
| `-o / --out-dir` | 覆盖配置中的 `file_directory` |
| `--enable-check` | 开启数据质量检查（超差则不输出文件） |
| `--scan` | 只扫描静止窗口，不执行标定 |
| `--scan-topk` | 扫描模式列出的候选个数（默认 10） |
| `--scan-step` | 扫描模式滑窗步长（秒，默认 1.0） |

## 配置说明

见 `config_example.yaml`，关键项：

- `imu_file` / `file_directory`：输入文件与输出目录
- `start_timestamp` / `end_timestamp`：标定时间范围，0 表示不限
- `qCI: [w, x, y, z]`：外参先验，只取其中 yaw 覆盖标定结果的 yaw（与 C++ 一致，roll/pitch 被忽略）
- `tCI: [x, y, z]`：平移，原样写入 `tci_*`
- `duration_time_ms`：标定所需最短数据时长（默认 15000 ms），窗口短于此值报 `CALIB_EOL_STILL_DATA_COLLECTING`
- `rpy_thred`：roll/pitch 超差阈值（默认 3°）。**若 IMU 安装倾角本身较大，需要调大此值或设置 `standard_roll/standard_pitch`**，否则会报 `CALIB_EOL_PITCH/ROLL_OUTOF_RANGE` 而不产出文件
- `enable_check` 及其余 `*_thred`：数据质量检查，默认关闭

## 与 C++ 输出的一致性说明

- 三个 JSON 与 `DataAdapter::SaveImuEolRes` 键名、取值、单位完全一致：`euler_angle_ci_*` 为**弧度**，`acc_bias_*` 为 m/s²，`gyro_bias_*` 为 rad/s。
- JSON 序列化对齐 `nlohmann::json::dump(4)`：键名字典序、4 空格缩进、结尾无换行。
- `imu_calibration_checksum.json` 的 `checksum` 为 CRC16/CCITT-FALSE，算在 `imu_calibration.json` 的写盘字节上。

差异（不影响标定数值）：

- 帧间时间戳统计使用浮点毫秒（C++ 用整数毫秒），仅影响丢帧率 / 时间戳检查项，不影响 bias / 姿态。
