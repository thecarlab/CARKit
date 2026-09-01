# OSRacer Base

<!-- markdownlint-disable MD013 MD033 -->

<p align="center">
  <a href="./README.md">English</a> ·
  <a href="./README_zh.md">简体中文</a>
</p>

<p align="center">
  <img src="docs/assets/readme/osracer-base-hero.jpg" alt="OSRacer Base ROS 2 底盘接口" width="100%">
</p>

<p align="center">
  <strong>OSRacer 软件平台的 ROS 2 底盘接口。</strong>
</p>

<p align="center">
  <a href="https://github.com/osrbot/osracer_base/actions/workflows/ros2-ci.yml"><img src="https://github.com/osrbot/osracer_base/actions/workflows/ros2-ci.yml/badge.svg" alt="ROS 2 CI"></a>
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros" alt="ROS 2 Humble"></a>
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros" alt="ROS 2 Jazzy"></a>
  <a href="./LICENSE"><img src="https://img.shields.io/badge/License-MIT-2EA44F" alt="MIT License"></a>
</p>

OSRacer Base 为 OSRacer 车辆控制器与上层机器人软件提供可复用的 ROS 2
连接。软件包将底盘串口数据转换为标准 ROS 消息，并接收速度控制和 Ackermann
控制命令。

## 功能特点

- 速度与 Ackermann 控制接口
- 里程计、IMU、遥控器原始通道、磁力计和电池状态发布
- 运动和惯性数据使用统一时间戳
- 自动适配轴距、前进/倒车速度限制、最大转角和电池显示范围
- 可配置 ROS 坐标系、发布选项、话题和协方差
- 命令超时自动停车
- 串口自动重连与连接状态诊断
- 启用运动命令前验证固件接口
- 适合车辆部署的稳定 udev 设备名称
- ROS 2 Humble 与 Jazzy 持续集成

## 最新版本

**OSRacer Base v0.3.0** 是当前 ROS 2 底盘接口版本，主要包括：

- 每次连接时读取并严格校验控制器报告的车辆能力；
- 在允许运动前执行故障关闭式固件接口检查；
- 使用统一时间戳发布同步里程计和惯性数据；
- 在发布 ROS 消息前拒绝非法数值遥测；
- 提供自动停止、串口重连和连接状态诊断；
- 覆盖 ROS 2 Humble 与 Jazzy 的构建和测试。

详见 [v0.3.0 版本说明](https://github.com/osrbot/osracer_base/releases/tag/v0.3.0)。

## 安装

### 作为 OSRacer 的组成部分

完整的 [OSRacer](https://github.com/osrbot/osracer) 工作空间通过
`osracer.repos` 导入兼容的 Base 版本。整车、SLAM、导航和竞速应用建议使用
这种方式。

### 独立工作空间

```bash
mkdir -p ~/osracer_base_ws/src
cd ~/osracer_base_ws/src
git clone https://github.com/osrbot/osracer_base.git

source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro humble -r -y

cd ~/osracer_base_ws
colcon build --symlink-install
source install/setup.bash
```

Ubuntu 24.04 与 ROS 2 Jazzy 环境应改用 `/opt/ros/jazzy/setup.bash` 和
`--rosdistro jazzy`。

## 设备配置

每台 Linux 系统只需安装一次 udev 规则：

```bash
ros2 run osracer_base install_udev_rules
```

安装后重新连接 USB。如果当前用户刚加入 `dialout` 用户组，请注销后重新登录。

无需启动 ROS 节点即可检查设备：

```bash
ros2 run osracer_base check_device
```

## 启动

直接启动驱动。驱动会在串口握手期间自动读取兼容控制器报告的车辆几何与运行限制：

```bash
ros2 launch osracer_base chassis_driver.launch.py
```

默认串口设备为 `/dev/osrbot_base`。仅在确有需要时修改：

```bash
ros2 launch osracer_base chassis_driver.launch.py \
  port:=/dev/ttyACM0
```

在 RViz 中查看里程计和 TF：

```bash
ros2 launch osracer_base odom_view.launch.py
```

## ROS 接口

### 订阅

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 线速度与角速度命令 |
| `/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | CARKit 速度与转角命令 |

驱动使用最近收到的命令。如果两个接口在 `cmd_timeout` 时间内都没有发布数据，
驱动将发送停车命令。

### 发布

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 底盘里程计 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 姿态、角速度与加速度 |
| `/rc_data` | `std_msgs/msg/Int32MultiArray` | 遥控器原始通道 |
| `/magnetometer_data` | `sensor_msgs/msg/MagneticField` | 磁场测量值 |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 电池电压与显示电量 |

遥控器、磁力计、电池状态和里程计 TF 可以分别启用或关闭。

## 配置

每次建立串口连接时，驱动都会依次验证 `fw version`、`profile get` 和
`vehicle get` 响应，全部通过后才允许运动。通过验证的能力合同提供轴距、独立的
前进与倒车速度限制、最大转角和电池显示范围。这些值只保存在当前连接的内存中，
重连后会重新读取。

ROS 坐标系、TF 发布、传感器发布、话题、协方差和串口时序仍通过 launch 参数
配置。控制器报告的车辆能力不作为 ROS 参数公开。

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `port` | `/dev/osrbot_base` | 底盘串口设备 |
| `baudrate` | `460800` | 串口波特率 |
| `cmd_timeout` | `0.5` | 未收到命令后自动停车的等待时间 |
| `reconnect_interval` | `2.0` | 串口重连间隔，单位为秒 |
| `odom_frame_id` | `odom` | 里程计坐标系 |
| `base_frame_id` | `base_footprint` | 车辆基础坐标系 |
| `imu_frame_id` | `imu_link` | IMU 坐标系 |
| `publish_tf` | `true` | 发布里程计 TF |
| `publish_rc` | `true` | 发布遥控器通道 |
| `publish_mag` | `true` | 发布磁场数据 |
| `publish_battery` | `true` | 发布电池状态 |

电池电压和显示范围由控制器提供。该范围只负责把电压换算为
`sensor_msgs/msg/BatteryState` 中显示的百分比，不会改变电压测量和车辆保护行为。

公开握手与校验规则见
[车辆能力协议](docs/vehicle_capability_contract_zh.md)。

## 控制示例

发布低速速度命令：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

发布 Ackermann 命令：

```bash
ros2 topic pub --once /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.3, steering_angle: 0.1}}"
```

查看电池数据：

```bash
ros2 topic echo /battery_state
```

第一次进行运动测试时，应当将驱动轮悬空，并确保能够随时紧急停止车辆。

## 兼容性

启动和每次重连时，驱动都会在启用数据流之前检查固件接口、可运动的 profile
状态和车辆能力合同。控制器必须支持 Proto 1.1 与 `vehicle get` Contract 1。
在 `modern` 模式下，不支持该命令的旧固件会保持断开且不会收到运动命令。

CARKit 也支持本车使用的旧版 OSRacer 控制器。该控制器发布独立的 `i`、`o`、
`m` 和 `r` 遥测帧，并且不支持 `fw version`、`profile get` 或 `vehicle get`。
必须显式选择 `protocol_mode:=legacy`；此模式使用 ROS 中配置的保守轴距、速度、
转角和电池限制。CARKit 的 `osracer_bringup/bringup_launch.py` 默认选择旧版模式。

适配较早的 OSRacer launch 文件时，应使用当前 Base 参数名称：

| 旧 launch 参数 | 当前 Base 参数 |
| --- | --- |
| `port_name` | `port` |
| `baud_rate` | `baudrate` |
| `odom_frame` | `odom_frame_id` |
| `base_frame` | `base_frame_id` |
| `imu_frame` | `imu_frame_id` |
| `cmd_watchdog_timeout_s` | `cmd_timeout` |
| `reconnect_interval_s` | `reconnect_interval` |
| `firmware_version_timeout_s` | `firmware_version_timeout` |
| `link_status_enabled` | `connection_status_enabled` |
| `link_ping_period_s` | `connection_refresh_period` |
| `mag_frame` | `mag_frame_id` |

## 故障处理

| 现象 | 建议处理方式 |
| --- | --- |
| 找不到 `/dev/osrbot_base` | 重新安装 udev 规则、连接 USB，并确认当前用户属于 `dialout` 用户组。 |
| 打开串口时权限不足 | 确认用户组设置，然后注销并重新登录。 |
| 串口被占用 | 停止正在使用底盘设备的其他 ROS 节点或工具。 |
| 驱动报告接口不匹配 | 安装支持 Proto 1.1 和车辆能力 Contract 1 响应的控制器固件。 |
| 命令无法驱动车辆 | 检查连接日志、遥控器优先级、命令话题与超时设置。 |
| 话题没有数据 | 运行 `check_device`，重启驱动并检查话题频率。 |

## 开发

运行软件包测试：

```bash
python3 -m pytest -q test
```

ROS 2 CI 会在 Humble 和 Jazzy 上构建并测试软件包。用户可见的版本变化记录在
[CHANGELOG.md](CHANGELOG.md) 中。

## 支持

- [GitHub Issues](https://github.com/osrbot/osracer_base/issues)
- [OSRacer 文档](https://github.com/osrbot/osracer)
- 技术支持与合作：[winter@osrbot.com](mailto:winter@osrbot.com)

## 作者

- Zhihao ZHANG
- Kit So
- Jintai WANG
- dajianli

## 许可证

OSRacer Base 使用 [MIT License](LICENSE) 开源。
