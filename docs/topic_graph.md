# Topic Graph Overview

This graph reflects the stable hardware-neutral CARKit workflow. The selected
OSRacer or F1TENTH adapter owns device details. `carkit_control_center` is the
final `/ackermann_cmd` publisher whenever unified control is enabled.

## Unified course flow

```text
selected chassis -> /odom + /camera/*
selected lidar -> /scan/raw -> carkit_scan_filter -> /scan

/camera/* -> reference|ADA Academy|Intro2AV perception -> /yolo/*
/odom + /goal_pose -> reference|ADA Academy|Intro2AV planning -> /plan
/odom + /plan -> reference|ADA Academy|Intro2AV control -> /drive

/teleop + /drive + /behavior/*
  -> carkit_control_center
  -> /ackermann_cmd
  -> selected chassis

C++ CARKit WebSocket bridge :9090 -> CARKit WebUI :8080
```

Launch all selected components with `carkit_bringup/carkit.launch.py`; course
profiles change implementations without changing topic names.

## Manual Driving And Mapping Control

```text
joystick
  -> joy_node
  -> /joy
  -> osracer joystick teleop
  -> /teleop
  -> osracer command relay
  -> /ackermann_cmd
  -> osracer_base
  -> vehicle

osracer_base
  -> /odom
  -> /imu/data
  -> /battery_state
```

Launch:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Use this direct path for manual driving, mapping, and vehicle checks.

## Nav2 Mapping

```text
sllidar_ros2
  -> /scan

/odom + /scan + base_link->laser TF
  -> SLAM Toolbox
  -> /map

/map
  -> nav2_map_server map_saver_cli
  -> /workspaces/CARKit/map/*.yaml + *.pgm
```

Launch human control first, then mapping:

```bash
ros2 launch carkit_human_control joystick.launch.py
ros2 launch carkit_navigation navigation.launch.py mode:=mapping
```

## Autonomous Navigation

In AV mode, remap the manual relay away from `/ackermann_cmd` so the control
center is the only final command publisher.

```text
joystick
  -> joy_node
  -> /joy
  -> osracer joystick teleop
  -> /teleop

sllidar_ros2
  -> /scan

osracer_base
  -> /odom

/scan + /odom + /initialpose + /workspaces/CARKit/map/<map>.yaml
  -> AMCL/Nav2
  -> map->odom TF
  -> /amcl_pose

Nav2 planner/controller
  -> /cmd_vel
  -> twist_to_ackermann
  -> /drive

/joy + /teleop + /drive + /behavior/*
  -> carkit_control_center
  -> /ackermann_cmd
  -> osracer_base
  -> vehicle
```

Launch:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/manual_command_unused
ros2 launch carkit_control_center control_center.launch.py
ros2 launch carkit_navigation navigation.launch.py \
  map:=/workspaces/CARKit/map/map.yaml
```

## Perception And Behavior

```text
OSRacer UVC color image
  -> /camera/camera/color/image_raw

OSRacer UVC camera info
  -> /camera/camera/color/camera_info

color image
  -> perception_2d_node
  -> /yolo/detections_2d
     (carkit_perception_msgs/msg/YoloDetection2DArray)
  -> carkit_behavior
```

Behavior inputs:

```text
/control_center/main_state
/yolo/detections_2d
/camera/camera/color/camera_info
/scan
  -> carkit_behavior
```

Behavior outputs:

```text
carkit_behavior
  -> /behavior/state
  -> /behavior/override_active
  -> /behavior/override_cmd
  -> /behavior/speed_limit
  -> /behavior/cone_obstacles
```

Control-center behavior integration:

```text
/behavior/override_active + /behavior/override_cmd
  -> carkit_control_center
  -> /ackermann_cmd

/behavior/speed_limit
  -> carkit_control_center
  -> clamps /drive speed in AUTO_DRIVE
```

Nav2 cone obstacle integration:

```text
/behavior/cone_obstacles
  -> local_costmap obstacle_layer cone source
  -> global_costmap obstacle_layer cone source
  -> Nav2 replanning around cones
```

## Final Command Ownership

```text
Manual/mapping:
  osracer command relay -> /ackermann_cmd

Autonomous driving:
  carkit_control_center -> /ackermann_cmd
```

Do not run both final publishers on `/ackermann_cmd` at the same time. For AV
driving, start `carkit_human_control` with
`vehicle_command_topic:=/manual_command_unused`.
