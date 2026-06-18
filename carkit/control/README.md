# Control

CARKit control is split into three Python packages:

- `carkit_human_control`: launches joystick teleop, VESC, odometry, and the
  vendored vehicle stack
- `carkit_behavior`: converts typed perception detections into behavior
  overrides, speed limits, and cone obstacles
- `carkit_control_center`: publishes the final `/ackermann_cmd` for autonomous
  driving

## Active Command Architecture

Manual driving and mapping use `carkit_human_control` directly:

```text
/joy -> joy_teleop -> /teleop
/teleop -> ackermann_mux -> /ackermann_cmd
/ackermann_cmd -> ackermann_to_vesc_node -> VESC motor and servo commands
VESC feedback -> vesc_to_odom_node -> /odom
```

Autonomous driving uses `carkit_control_center` as the final arbiter:

```text
/joy -> joy_teleop -> /teleop
Nav2 -> /cmd_vel -> twist_to_ackermann -> /drive
/yolo/detections_2d -> carkit_behavior -> /behavior/*
/teleop + /drive + /behavior/* + /joy -> carkit_control_center -> /ackermann_cmd
/ackermann_cmd -> ackermann_to_vesc_node -> VESC motor and servo commands
VESC feedback -> vesc_to_odom_node -> /odom
```

Nav2 should run with `start_command_mux:=false`, which is the default in the
current launch files. In autonomous driving, launch `carkit_human_control` with
`vehicle_command_topic:=/ackermann_mux_unused` so the legacy mux does not also
publish `/ackermann_cmd`.

## Bringup

For manual driving, mapping, and vehicle checks:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

For autonomous driving, start joystick, VESC, and odometry with the legacy mux
output remapped away:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/ackermann_mux_unused
```

Start the autonomous command arbiter:

```bash
ros2 launch carkit_control_center control_center.launch.py
```

The control center starts in `HUMAN_CONTROL`, follows fresh `/teleop`, and
publishes zero if teleop commands go stale. Use the joystick buttons to switch
to `AUTO_DRIVE` or `EMERGENCY_STOP`.

Start Nav2 for autonomous driving:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  map:=/workspaces/CARKit/map/map.yaml
```

Start the behavior layer when perception is running:

```bash
ros2 launch carkit_behavior behavior_center.launch.py
```

## Control Center

`carkit_control_center` subscribes to:

- `/joy` (`sensor_msgs/Joy`)
- `/teleop` (`ackermann_msgs/AckermannDriveStamped`)
- `/drive` (`ackermann_msgs/AckermannDriveStamped`)
- `/behavior/override_active` (`std_msgs/Bool`)
- `/behavior/override_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/behavior/speed_limit` (`std_msgs/Float32`)

It publishes:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/control_center/main_state` (`std_msgs/String`)
- `/control_center/selected_cmd` (`std_msgs/String`)
- `/control_center/debug` (`std_msgs/String`)

Main states:

- `HUMAN_CONTROL`: publishes fresh `/teleop`, otherwise zero
- `AUTO_DRIVE`: publishes fresh behavior override, otherwise fresh `/drive`
- `EMERGENCY_STOP`: always publishes zero

Joystick buttons are edge-triggered:

- `auto_button`: enter `AUTO_DRIVE`
- `human_button`: enter `HUMAN_CONTROL`
- `estop_button`: enter `EMERGENCY_STOP`
- `clear_estop_button`: clear emergency stop and return to `HUMAN_CONTROL`

Defaults live in `carkit_control_center/config/control_center.yaml`.

## Behavior Layer

`carkit_behavior` subscribes to:

- `/control_center/main_state` (`std_msgs/String`)
- `/yolo/detections_2d`
  (`carkit_perception_msgs/msg/YoloDetection2DArray`)
- `/scan` (`sensor_msgs/LaserScan`)
- `/camera/camera/color/camera_info` (`sensor_msgs/CameraInfo`)

It publishes:

- `/behavior/state` (`std_msgs/String`)
- `/behavior/override_active` (`std_msgs/Bool`)
- `/behavior/override_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/behavior/speed_limit` (`std_msgs/Float32`)
- `/behavior/cone_obstacles` (`sensor_msgs/PointCloud2`)

Behavior logic only runs while the control center state is `AUTO_DRIVE`.
Priority is stop sign, traffic light, cone, then normal Nav2.

- Stop signs publish a zero override for the configured stop duration.
- Red/yellow traffic lights publish a zero override while fresh.
- Stop signs and cones use their 2D bearing with lidar for metric range.
- Camera/lidar fusion accounts for the camera being mounted 0.08 m forward of
  the lidar. The camera's 0.08 m lower mounting height cannot be corrected
  from a planar scan, so cones must intersect the lidar plane to be localized.
- Nearby red/yellow lights stop the car; green releases the override. The
  configurable bounding-box height ratio filters distant lights.
- Cone detections publish lidar-frame PointCloud2 obstacles and a temporary
  speed limit; Nav2 handles steering by replanning around them.

Defaults live in `carkit_behavior/config/behavior_center.yaml`.

## Human-Control Launch Arguments

- `joy_config`: joystick device, axis mapping, deadzone, speed, and steering
  configuration. Defaults to
  `carkit/vehicle/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`.
- `vesc_config`: VESC port, calibration, limits, wheelbase, and odometry
  configuration. Defaults to
  `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`.
- `mux_config`: legacy Ackermann mux config.
- `vehicle_command_topic`: legacy mux output topic. Defaults to
  `/ackermann_cmd` for direct human control. Use `/ackermann_mux_unused` when
  `carkit_control_center` owns `/ackermann_cmd` for autonomous driving.

## Verify

```bash
ros2 topic echo /joy --once
ros2 topic echo /teleop --once
ros2 topic echo /control_center/main_state --once
ros2 topic echo /control_center/selected_cmd --once
ros2 topic echo /ackermann_cmd --once
ros2 topic echo /odom --once
```

In `AUTO_DRIVE`, also check:

```bash
ros2 topic echo /drive --once
ros2 topic echo /behavior/state --once
ros2 topic echo /behavior/cone_obstacles --once
```
