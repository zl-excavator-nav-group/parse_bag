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

# 点云 / 图像按帧 dump：推荐在脚本顶部配置（见「点云 / 图像按帧 dump」节），
# 也可用命令行覆盖配置（--dump-media / --pointcloud-topics / --image-topics 等）
python3 parse_bag.py --bag <bag目录> --dump-media \
    --pointcloud-topics /livox/lidar_192_168_1_158 \
    --image-topics /cam0/image_raw/compressed \
    --start-time 1787218155 --end-time 1787218160 --img-format png
```

> 结果统一写到 `<out>/<bag目录名>_parsed_res/` 下，例如
> `/path/to/out/rosbag2_2026_08_20-09_24_26_parsed_res/`（`--out` 是根目录，不是最终目录）。

## topic 过滤规则

- 默认按**关键字**过滤：topic 名（小写）包含任一关键字才解析。
- 关键字列表在脚本顶部 `KEYWORDS` 里配置，也可用 `--keywords` 覆盖。
- 用 `--topics` 传入精确 topic 名时，忽略关键字过滤。
- 点云 `/livox/lidar_*`、图像 `/cam*/image_raw/compressed` 因不含默认关键字而自动跳过；
  需要 dump 时在脚本顶部配置 `POINTCLOUD_TOPICS` / `IMAGE_TOPICS`（或用
  `--pointcloud-topics` / `--image-topics`），并打开 `DUMP_MEDIA` 开关（见下节）。

## 输出格式

每个匹配的 topic 生成一个文件，文件名由 topic 名而来：`/gps/pos` → `gps_pos.txt`。
每行空格分隔，第一列是时间戳（秒，精确到纳秒，取 bag 记录的消息接收时间）。
默认不带表头；加 `--header` 会在第一行写列名 + 单位。

按消息类型定制的列（不含第一列时间戳）：

| 消息类型 | 列 | 说明 |
| --- | --- | --- |
| `sensor_msgs/msg/Imu`（IMU） | `acc_x acc_y acc_z gyro_x gyro_y gyro_z` | 加速度统一 m/s²，角速度 rad/s |
| `nmea_msgs/msg/Gpgga` | `lat lon alt` | 经纬度（deg）+ 高程（m） |
| `gnss_comm/msg/GnssPosMsg` | `lat lon alt pos_type` | 经纬度 + 高程 + 解算状态（SolStatus） |
| `gnss_comm/msg/GinsPvaMsg` | `lat lon alt roll pitch yaw vel_e vel_n vel_u vel_yaw` | 经纬度 + 姿态角 + 速度；另生成 `*_utm.txt` / `*_vel_utm.txt`（TUM）与 `*.geojson` |
| `gnss_comm/msg/GnssHdgMsg` | `hdg_type baseline heading pitch` | 航向解状态 + 基线 + 航向/俯仰；与位置 topic 匹配后另生成 `*_hdg_utm.txt` / `*_hdg_odom.txt` / `*.geojson` |
| `gnss_comm/msg/GnssVelMsg` | `vel_e vel_n vel_u` | ENU 速度 |
| `livox_ros_driver2/msg/CustomMsg`（点云） | `header_time timebase point_num lidar_id` | 元信息（不含点云本体）；第一列是 header 时间 |
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
度转弧度再经 `ENUYawToUTMYaw` 修正 UTM 网格收敛角。

在 `gps_pos_hdg_utm.txt` 之上还会再出两份产物：

- `gps_pos_hdg_odom.txt`：把上面的 TUM 轨迹**对齐到起点坐标系**（同样是 TUM 格式）。
  以首帧位姿为基准，roll/pitch 置 0、只保留首帧 yaw 构成 `T_start`，每帧做
  `inv(T_start) @ T`，于是首帧平移为 `0 0 0`、姿态为单位阵，后续帧是相对起点的位姿。
  （PVA 链路的同类产物叫 `gnss_odom.txt`，两者互不覆盖。）
- `gps_pos.geojson`：轨迹的 GeoJSON，直接由经纬度原文件 `gps_pos.txt` 生成（TUM 里是
  UTM 坐标，不能直接喂 `utils.create_geojson`）。Point 的 `FeatureCollection`，
  `geometry` 为 `Point([lon, lat])`，`properties` 为 `{"time": 时间戳, "info": alt}`，
  按 `ts_interval=0.1` 秒抽稀。因为取的是位置原文件，帧数会比 TUM 多出那几帧
  「没匹配到航向」的位置数据。

> 生成 TUM 依赖同目录下的 `utils.py`（以及 `utm`、`geojson` 等第三方包）。如未安装，
> 可执行 `python3 -m pip install --user utm geojson`；缺少依赖时脚本仍会正常导出其它
> 消息，只是跳过 TUM 生成。

**PVA 的 GeoJSON 输出**：`GinsPvaMsg` 还会把轨迹导出为 `gps_pva.geojson`（对应
`run_parse_mcap.py` 里的 `utils.create_geojson(llh_file, ts_interval=0.1)`）。它是一个
Point 的 `FeatureCollection`，每个点 `geometry` 为 `Point([lon, lat])`，`properties`
为 `{"time": 时间戳, "info": 最后一列（即 vel_yaw 航向角，单位度）}`。为避免轨迹点
过密，按 `ts_interval=0.1` 秒做了时间抽稀（本 bag 的 pva 约 46Hz，抽稀后约 10Hz）。

## 点云 / 图像按帧 dump

由脚本顶部的一键开关 `DUMP_MEDIA` 控制：**关闭**（默认）时，点云 topic 只写元信息
txt，图像不导出；**打开**时，`POINTCLOUD_TOPICS` / `IMAGE_TOPICS` 里列出的 topic 会
按帧落盘成独立文件，目录名即 topic 名：

```
<out>/<bag名>_parsed_res/
  livox_lidar_192_168_1_158.txt        # 点云元信息（始终生成）
  livox_lidar_192_168_1_158/           # 点云（DUMP_MEDIA=True 时每帧 .pcd）
    1787218156.400168210.pcd
    ...
  cam0_image_raw_compressed/           # 图像（DUMP_MEDIA=True 时每帧 .jpg/.png）
    1787218158.247785631.jpg
    ...
```

脚本顶部配置项（`--pointcloud-topics` / `--image-topics` / `--start-time` /
`--end-time` / `--img-format` / `--dump-media` / `--no-dump-media` 可覆盖）：

```python
DUMP_MEDIA = False                     # 一键开关
POINTCLOUD_TOPICS = []                 # 例: ["/livox/lidar_192_168_1_158"]
IMAGE_TOPICS = []                      # 例: ["/cam0/image_raw/compressed"]
DUMP_START_TIME = None                 # 时间范围起点（秒，epoch）；None 不限
DUMP_END_TIME = None                   # 时间范围终点
DUMP_IMG_FORMAT = "jpg"                # 图像格式 jpg / png
```

- 按**消息类型识别**：点云（`livox_ros_driver2/msg/CustomMsg`、
  `sensor_msgs/msg/PointCloud2`）→ `.pcd`；图像（`sensor_msgs/msg/Image`、
  `sensor_msgs/msg/CompressedImage`）→ `.jpg`/`.png`。
- 文件名用消息 **header.stamp**（`秒.纳秒`），同时间戳多帧会自动加 `_1/_2/...` 后缀。
- **PCD**：二进制格式，字段 `x y z intensity`，其中 `intensity` = Livox 的 `reflectivity`
  （0~255）。`PointCloud2` 会额外保留 `intensity` 字段（若存在）。
- **图像**：`DUMP_IMG_FORMAT` 选 `jpg`（默认）或 `png`。`CompressedImage` 源格式与目标
  一致时直接复制原始字节（无损、更快），否则解码后重编码。
- **时间范围**：`DUMP_START_TIME` / `DUMP_END_TIME`（单位秒，epoch，可含小数），用
  header.stamp 过滤，只 dump 落在闭区间内的帧；两者都可省略。

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
| `--dump-media` / `--no-dump-media` | 开/关点云图像 dump，覆盖 `DUMP_MEDIA` | 脚本顶部 `DUMP_MEDIA` |
| `--pointcloud-topics` | 逗号分隔的点云 topic，覆盖 `POINTCLOUD_TOPICS` | 脚本顶部配置 |
| `--image-topics` | 逗号分隔的图像 topic，覆盖 `IMAGE_TOPICS` | 脚本顶部配置 |
| `--start-time` | 时间范围起点（秒，epoch），只 dump 此后的点云/图像帧 | `DUMP_START_TIME` |
| `--end-time` | 时间范围终点（秒，epoch），只 dump 此前的点云/图像帧 | `DUMP_END_TIME` |
| `--img-format` | 图像输出格式 `jpg`/`png` | `DUMP_IMG_FORMAT` |

## 录制 bag

`record_bag.py` 读取 `record_config.yaml`，运行 `ros2 bag record`；
`parse_bag.py` 用于解析已有 bag，支持 MCAP 和 SQLite3。

```bash
python3 record_bag.py --config record_config.yaml
```

若报 `invalid choice: 'mcap' (choose from 'sqlite3', ...)`，说明当前 ROS2 环境
未识别到 MCAP 存储插件。对于通过 apt 安装的 ROS2，在录制机器上执行
（先 source 对应发行版，确保 `ROS_DISTRO` 已设置）：

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap
source /opt/ros/${ROS_DISTRO}/setup.bash
# 然后 source 你的工作空间 install/setup.bash
ros2 bag record --help  # 确认 -s 的可选值包含 mcap
```

也可将 `record_config.yaml` 的 `storage` 改为 `"sqlite3"` 后录制 `.db3`，
脚本会自动忽略 MCAP 专用的 `storage_preset_profile`。
`storage_config_file` 若非空仍会传给所选插件，切换格式时需清空或换成对应格式的配置。
脚本在创建输出目录之前检查插件；若指定插件不可用，会报错并给出处理提示。

## 说明

- 消息类型无法解析（缺包）的 topic 会被跳过，并在 stderr 打印告警。
- 0 条消息的 topic（如本 bag 的 `/gps/imu_raw`、`/gps/vel`）不会生成文件。
- 全量解析约 17 万条消息（IMU 占大头）需几十秒到几分钟；用 `--topics` 只导需要的 topic 会更快。
