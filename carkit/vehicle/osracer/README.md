# CARKit OSRacer Integration

This directory replaces the former F1TENTH/VESC vehicle stack. CARKit keeps
its existing ROS contracts while the OSRacer side adapts to them:

```text
/joy -> osracer joystick teleop -> /teleop
/teleop -> osracer command relay -> /ackermann_cmd
/ackermann_cmd (AckermannDriveStamped) -> osracer_base -> serial chassis
serial chassis -> /odom + /imu/data + /magnetometer_data + /battery_state
LakiBeam Ethernet lidar -> /scan
icSpring UVC camera -> latest-frame C++ capture -> /camera/camera/color/image_raw/compressed
```

`osracer_base` is derived from OSRacer Base v0.3.0. Its modern Proto 1.1
capability handshake is retained, and CARKit adds an explicit legacy mode for
the connected `303a:1001` controller. The legacy controller was verified to
accept `v <m/s> <steering-degrees>` commands and publish separate `i`, `o`,
`m`, and `r` telemetry frames.

Prepare the host device before starting Docker:

```bash
./docker/setup_osracer_device.sh
./docker/run_jetson.sh
```

Inside Docker, build and launch:

```bash
./docker/build_workspace.sh
source install/setup.bash
ros2 run osracer_base check_device
ros2 launch carkit_human_control joystick.launch.py
ros2 launch osracer_bringup sensors_launch.py
```

The OSRacer sensor launch uses the chassis' LakiBeam at `192.168.8.2` and the
stable `/dev/osrbot_usb_cam` camera alias. Its topics match CARKit's existing
sensor contracts, so CARKit perception and navigation do not need renaming.
The camera is drained at 30 FPS to prevent kernel/driver backlog, while its
public image and camera-info topics remain fixed at the course requirement of
10 Hz. Raw BGR images remain available on `/camera/camera/color/image_raw` and
are decoded only when that topic has a subscriber.

The default controller mode is `legacy`. After a firmware upgrade that
implements Proto 1.1 and the vehicle capability contract, launch with
`protocol_mode:=modern`.

For first motion tests, keep the path clear, use a conservative speed, and
keep the emergency stop within reach.
