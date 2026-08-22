# parse_bag —— ROS2 bag 转文本

把 ROS2 bag 里的消息解析出来，按 topic 分别写成一个 `.txt` 文件（空格分隔，
默认**不带表头**，第一列是时间戳）。

## 运行前提

脚本依赖 `rosbag2_py` / `rclpy` / `rosidl_runtime_py`，并且需要能 import 到
bag 里出现的自定义消息（本工作空间 `~/ws_excavator_nav` 里定义的 `gnss_comm`、
`livox_ros_driver2` 等）。运行前在同一个终端里 source 好环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_excavator_nav/install/setup.bash
```

> 只要环境 source 对了，脚本放在任意目录都能跑，不必放在 ROS 工作空间内。

## 用法

```bash
# 先看看 bag 里有哪些 topic 和消息类型
python3 parse_bag.py --bag <bag目录> --list-topics

# 用关键字过滤导出（默认关键字 gnss,imu,encoder,gps）
python3 parse_bag.py --bag <bag目录>

# 自定义关键字
python3 parse_bag.py --bag <bag目录> --keywords gnss,imu,encoder,gps

# 精确指定某几个 topic（传入后忽略关键字过滤）
python3 parse_bag.py --bag <bag目录> --topics /gps/pos,/gps/pva

# 指定输出根目录 / 写表头行
python3 parse_bag.py --bag <bag目录> --out /path/to/out --header
```

> 结果统一写到 `<out>/<bag目录名>_parsed_res/` 下，例如
> `/path/to/out/rosbag2_2026_08_20-09_24_26_parsed_res/`（`--out` 是根目录，不是最终目录）。

## topic 过滤规则

- 默认按**关键字**过滤：topic 名（小写）包含任一关键字才解析。
- 关键字列表在脚本顶部 `KEYWORDS` 里配置，也可用 `--keywords` 覆盖。
- 用 `--topics` 传入精确 topic 名时，忽略关键字过滤。
- 点云 `/livox/lidar_*`、图像 `/cam3/image_raw/compressed` 因不含关键字而自动跳过。

## 输出格式

每个匹配的 topic 生成一个文件，文件名由 topic 名而来：`/gps/pos` → `gps_pos.txt`。
每行空格分隔，第一列是时间戳（秒，精确到纳秒，取 bag 记录的消息接收时间）。
默认不带表头；加 `--header` 会在第一行写列名 + 单位。

按消息类型定制的列（不含第一列时间戳）：

| 消息类型 | 列 | 说明 |
| --- | --- | --- |
| `sensor_msgs/msg/Imu`（IMU） | `acc_x acc_y acc_z gyro_x gyro_y gyro_z` | 加速度统一 m/s²，角速度 rad/s |
| `nmea_msgs/msg/Gpgga` | `lat lon alt` | 经纬度（deg）+ 高程（m） |
| `gnss_comm/msg/GnssPosMsg` | `lat lon alt` | 经纬度 + 高程 |
| `gnss_comm/msg/GinsPvaMsg` | `lat lon alt roll pitch yaw vel_e vel_n vel_u vel_yaw` | 经纬度 + 姿态角 + 速度；另生成 `*_utm.txt` / `*_vel_utm.txt`（TUM）与 `*.geojson` |
| `gnss_comm/msg/GnssHdgMsg` | `hdg_type baseline heading pitch` | 航向解状态 + 基线 + 航向/俯仰 |
| `gnss_comm/msg/GnssVelMsg` | `vel_e vel_n vel_u` | ENU 速度 |
| 其它类型 | 通用展平：递归展开所有标量字段 | 见下 |

**加速度单位换算**：livox 驱动的 IMU，`linear_acceleration` 单位是 **g**（源码注释
`lidar_imu_data_queue.h: Unit:g`），脚本会对 topic 名含 `livox` 的 IMU 自动 ×9.80665
换算到 m/s²；其它驱动默认按 m/s² 处理。换算规则在脚本顶部 `ACC_UNIT_TO_MPS2` 里配置。

**PVA 的 TUM 输出**：`GinsPvaMsg` 除生成 llh + 姿态 + 速度的 `gps_pva.txt` 外，还会把
经纬度/姿态转到 UTM 坐标系，生成**两份** TUM 文件：

- `gps_pva_utm.txt`：姿态用车身 yaw（`msg.yaw`，roll/pitch 取原始值）
- `gps_pva_vel_utm.txt`：姿态用速度航向（`msg.vel_yaw`，roll/pitch 置 0）

TUM 格式：

```
timestamp tx ty tz qx qy qz qw
```

其中 `tx/ty/tz` 为 UTM easting / northing / 高程（米），`qx qy qz qw` 为姿态四元数。
UTM 分区取第一条数据的 zone 后全程固定。姿态角（roll/pitch/yaw）原始单位是**度**，
脚本会先转成**弧度**再参与 yaw 的 UTM 网格收敛角修正和四元数计算（对应
`run_parse_mcap.py` 461~468 行调用 `utils.convert_llh_file_to_tum` 的逻辑）。

**GNSS 位置 + 航向的 TUM 输出**：当 bag 里同时有位置 topic（`GnssPosMsg`/`Gpgga`，
即 `gps_pos.txt`）和航向 topic（`GnssHdgMsg`，即 `gps_hdg.txt`）时，脚本会把两者按
**时间戳就近匹配**，输出 `gps_pos_hdg_utm.txt`（TUM 格式）。位置用经纬度转 UTM
easting/northing，高程取 alt；姿态取 hdg 的 `heading`（yaw）与 `pitch`，`roll` 置 0。
`GnssHdgMsg.heading` 定义是 *counterclockwise from east [deg]*（即 ENU 系 yaw），先
度转弧度再经 `ENUYawToUTMYaw` 修正 UTM 网格收敛角。也可脱离 bag 单独对已导出的
txt 运行：

```bash
python3 combine_pos_hdg_tum.py --dir output
# 或 python3 combine_pos_hdg_tum.py --pos output/gps_pos.txt --hdg output/gps_hdg.txt --out output/gps_traj_utm.txt
```

> 生成 TUM 依赖同目录下的 `utils.py`（以及 `utm`、`geojson` 等第三方包）。如未安装，
> 可执行 `python3 -m pip install --user utm geojson`；缺少依赖时脚本仍会正常导出其它
> 消息，只是跳过 TUM 生成。

**PVA 的 GeoJSON 输出**：`GinsPvaMsg` 还会把轨迹导出为 `gps_pva.geojson`（对应
`run_parse_mcap.py` 里的 `utils.create_geojson(llh_file, ts_interval=0.1)`）。它是一个
Point 的 `FeatureCollection`，每个点 `geometry` 为 `Point([lon, lat])`，`properties`
为 `{"time": 时间戳, "info": 最后一列（即 vel_yaw 航向角，单位度）}`。为避免轨迹点
过密，按 `ts_interval=0.1` 秒做了时间抽稀（本 bag 的 pva 约 46Hz，抽稀后约 10Hz）。

## 通用展平（其它消息类型）

对于没有单独定制的类型，脚本递归展开所有标量字段为 `父字段.子字段` 形式的多列
（如 `header.stamp.sec`、`time.tow`、`heading`），并**跳过数组**（避免点云/图像数据爆炸）。
若你需要为这些类型定制专用格式，告诉我列顺序即可。

## 命令行参数

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `--bag` | bag 目录路径 | 脚本顶部 `BAG_PATH` |
| `--out` | 输出目录 | 脚本顶部 `OUTPUT_DIR` |
| `--keywords` | 逗号分隔的关键字，匹配才解析；留空则全量 | `gnss,imu,encoder,gps` |
| `--topics` | 精确 topic 名，逗号分隔，可重复传入 | — |
| `--storage` | 存储格式 `sqlite3`/`mcap` | 自动识别 |
| `--sep` | 列分隔符 | 单个空格 |
| `--header` | 写表头行（默认不写） | 不写 |
| `--list-topics` | 只列出 topic 与类型后退出 | — |

## 说明

- 消息类型无法解析（缺包）的 topic 会被跳过，并在 stderr 打印告警。
- 0 条消息的 topic（如本 bag 的 `/gps/imu_raw`、`/gps/vel`）不会生成文件。
- 全量解析约 17 万条消息（IMU 占大头）需几十秒到几分钟；用 `--topics` 只导需要的 topic 会更快。
