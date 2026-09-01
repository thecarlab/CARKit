# OSRacer Base

<!-- markdownlint-disable MD013 MD033 -->

<p align="center">
  <a href="./README.md">English</a> ·
  <a href="./README_zh.md">简体中文</a>
</p>

<p align="center">
  <img src="docs/assets/readme/osracer-base-hero.jpg" alt="OSRacer Base ROS 2 chassis interface" width="100%">
</p>

<p align="center">
  <strong>The ROS 2 chassis interface for the OSRacer software platform.</strong>
</p>

<p align="center">
  <a href="https://github.com/osrbot/osracer_base/actions/workflows/ros2-ci.yml"><img src="https://github.com/osrbot/osracer_base/actions/workflows/ros2-ci.yml/badge.svg" alt="ROS 2 CI"></a>
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros" alt="ROS 2 Humble"></a>
  <a href="https://docs.ros.org/en/jazzy/"><img src="https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros" alt="ROS 2 Jazzy"></a>
  <a href="./LICENSE"><img src="https://img.shields.io/badge/License-MIT-2EA44F" alt="MIT License"></a>
</p>

OSRacer Base provides the reusable ROS 2 connection between an OSRacer vehicle
controller and the rest of the robot software. It translates the chassis serial
stream into standard ROS messages and accepts both velocity and Ackermann drive
commands.

## Features

- Velocity and Ackermann command interfaces
- Odometry, IMU, raw RC, magnetometer, and battery-state publication
- Shared timestamps for synchronized motion and inertial data
- Automatic wheelbase, directional speed-limit, steering-limit, and battery-range adaptation
- Configurable ROS frames, publication options, topics, and covariances
- Command timeout with automatic stop
- Serial reconnection and connection-state diagnostics
- Firmware-interface validation before motion commands are enabled
- Stable udev device naming for vehicle deployment
- ROS 2 Humble and Jazzy continuous integration

## Latest Release

**OSRacer Base v0.3.0** provides the current ROS 2 chassis interface with:

- controller-reported vehicle capabilities validated on every connection;
- fail-closed firmware-interface checks before motion is enabled;
- synchronized odometry and inertial publication with shared timestamps;
- rejection of invalid numeric telemetry before ROS message publication;
- automatic stop, serial reconnection, and connection-state diagnostics;
- build and test coverage on ROS 2 Humble and Jazzy.

See the [v0.3.0 release notes](https://github.com/osrbot/osracer_base/releases/tag/v0.3.0).

## Installation

### As part of OSRacer

The main [OSRacer](https://github.com/osrbot/osracer) workspace imports the
compatible Base revision through `osracer.repos`. This is the recommended path
for complete vehicle, SLAM, navigation, and racing applications.

### Standalone workspace

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

Use `/opt/ros/jazzy/setup.bash` and `--rosdistro jazzy` on Ubuntu 24.04 with
ROS 2 Jazzy.

## Device Setup

Install the udev rule once on each Linux system:

```bash
ros2 run osracer_base install_udev_rules
```

Reconnect USB after installation. If the current user was newly added to the
`dialout` group, log out and back in before starting the driver.

Check the device without starting the ROS node:

```bash
ros2 run osracer_base check_device
```

## Launch

Start the driver. Vehicle geometry and operating limits are read automatically
from a compatible controller during the serial handshake:

```bash
ros2 launch osracer_base chassis_driver.launch.py
```

The default serial device is `/dev/osrbot_base`. Override it only when required:

```bash
ros2 launch osracer_base chassis_driver.launch.py \
  port:=/dev/ttyACM0
```

View odometry and TF in RViz:

```bash
ros2 launch osracer_base odom_view.launch.py
```

## ROS Interfaces

### Subscriptions

| Topic | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear and angular velocity command |
| `/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | CARKit speed and steering-angle command |

The most recent command is applied. If neither interface publishes within
`cmd_timeout`, the driver sends a stop command.

### Publications

| Topic | Type | Purpose |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | Chassis odometry |
| `/imu/data` | `sensor_msgs/msg/Imu` | Orientation, angular velocity, and acceleration |
| `/rc_data` | `std_msgs/msg/Int32MultiArray` | Raw receiver channels |
| `/magnetometer_data` | `sensor_msgs/msg/MagneticField` | Magnetic-field measurement |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Battery voltage and display percentage |

RC, magnetometer, battery, and odometry TF publication can be enabled or
disabled independently.

## Configuration

On every serial connection, the driver validates `fw version`, `profile get`,
and `vehicle get` responses before enabling motion. The accepted capability
contract supplies wheelbase, separate forward and reverse speed limits, maximum
steering angle, and the battery display range. These values remain in memory
for that connection and are read again after reconnecting.

ROS frame names, TF publication, sensor publication, topics, covariances, and
serial timing remain configurable through launch arguments. Controller-reported
vehicle capabilities are not ROS parameters.

Frequently used parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `port` | `/dev/osrbot_base` | Chassis serial device |
| `baudrate` | `460800` | Serial baud rate |
| `cmd_timeout` | `0.5` | Time without a command before automatic stop |
| `reconnect_interval` | `2.0` | Serial reconnect interval in seconds |
| `odom_frame_id` | `odom` | Odometry frame |
| `base_frame_id` | `base_footprint` | Vehicle base frame |
| `imu_frame_id` | `imu_link` | IMU frame |
| `publish_tf` | `true` | Publish odometry TF |
| `telemetry_publish_rate_hz` | `50.0` | Maximum ROS publication rate for synchronized odometry and IMU frames |
| `imu_publish_rate_hz` | `30.0` | Maximum ROS publication rate for legacy high-rate IMU frames |
| `publish_rc` | `true` | Publish receiver channels |
| `publish_mag` | `true` | Publish magnetic-field data |
| `publish_battery` | `true` | Publish battery state |

The battery voltage and display range come from the controller. The range only
converts voltage to the percentage shown in `sensor_msgs/msg/BatteryState`; it
does not change voltage measurement or vehicle protection behavior.

See the [vehicle capability protocol](docs/vehicle_capability_contract.md) for
the public handshake and validation contract.

## Command Examples

Publish a low-speed velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Publish a CARKit-compatible Ackermann command:

```bash
ros2 topic pub --once /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.3, steering_angle: 0.1}}"
```

Inspect battery data:

```bash
ros2 topic echo /battery_state
```

Perform first motion tests with the driven wheels raised and an emergency stop
within reach.

## Compatibility

At startup and after every reconnect, the driver checks the firmware interface,
motion-ready profile state, and vehicle capability contract before enabling the
data stream. The controller must support Proto 1.1 and `vehicle get` Contract 1.
In `modern` mode, older firmware without this command remains disconnected and
motion commands are not sent.

CARKit also supports the earlier OSRacer controller found on this vehicle. It
publishes separate `i`, `o`, `m`, and `r` telemetry frames and does not expose
`fw version`, `profile get`, or `vehicle get`. This mode must be selected
explicitly with `protocol_mode:=legacy`; it uses conservative ROS-configured
wheelbase, speed, steering, and battery limits. The CARKit
`osracer_bringup/bringup_launch.py` launch selects legacy mode by default.

Use `protocol_mode:=modern` after installing firmware that implements Proto
1.1. Modern mode retains the fail-closed capability handshake described above.

When adapting an older OSRacer launch file, use the current Base parameter
names:

| Earlier launch name | Current Base parameter |
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

## Troubleshooting

| Symptom | Recommended action |
| --- | --- |
| `/dev/osrbot_base` is missing | Reinstall the udev rule, reconnect USB, and verify `dialout` membership. |
| Permission denied when opening the port | Confirm group membership, then log out and back in. |
| The port is busy | Stop any other ROS node or utility using the chassis device. |
| The driver reports an interface mismatch | Install controller firmware that supports Proto 1.1 and the vehicle capability Contract 1 response. |
| Commands do not move the vehicle | Check connection logs, receiver-control priority, command topic, and timeout. |
| A topic contains no data | Run `check_device`, restart the driver, and inspect the topic rate. |

## Development

Run the package tests:

```bash
python3 -m pytest -q test
```

ROS 2 CI builds and tests the package on Humble and Jazzy. See
[CHANGELOG.md](CHANGELOG.md) for user-visible changes.

## Support

- [GitHub Issues](https://github.com/osrbot/osracer_base/issues)
- [OSRacer documentation](https://github.com/osrbot/osracer)
- Technical support and collaboration: [winter@osrbot.com](mailto:winter@osrbot.com)

## Authors

- Zhihao ZHANG
- Kit So
- Jintai WANG
- dajianli

## License

OSRacer Base is released under the [MIT License](LICENSE).
