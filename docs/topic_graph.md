# Topic Graph Overview

This graph reflects the current CARKit workflow. Mapping and manual driving use
direct human control through the legacy mux. Autonomous driving uses
`carkit_control_center` as the final `/ackermann_cmd` publisher.

## Manual Driving And Mapping Control

```text
joystick
  -> joy_node
  -> /joy
  -> joy_teleop
  -> /teleop
  -> ackermann_mux
  -> /ackermann_cmd
  -> ackermann_to_vesc_node
  -> vesc_driver_node
  -> vehicle

vesc_driver_node
  -> vesc_to_odom_node
  -> /odom
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

In AV mode, remap the legacy mux away from `/ackermann_cmd` so the control
center is the only final command publisher.

```text
joystick
  -> joy_node
  -> /joy
  -> joy_teleop
  -> /teleop

sllidar_ros2
  -> /scan

vesc_driver_node
  -> vesc_to_odom_node
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
  -> ackermann_to_vesc_node
  -> vesc_driver_node
  -> vehicle
```

Launch:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/ackermann_mux_unused
ros2 launch carkit_control_center control_center.launch.py
ros2 launch carkit_navigation navigation.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  map:=/workspaces/CARKit/map/map.yaml
```

## Perception And Behavior

```text
RealSense color image
  -> /camera/camera/color/image_raw

RealSense aligned depth
  -> /camera/camera/aligned_depth_to_color/image_raw

RealSense aligned camera info
  -> /camera/camera/aligned_depth_to_color/camera_info

color + aligned depth + aligned camera_info
  -> perception_3d_node
  -> /yolo/detections_3d
     (carkit_perception_msgs/msg/YoloDetection3DArray)
  -> carkit_behavior
```

Behavior inputs:

```text
/control_center/main_state
/yolo/detections_3d
/odom
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
  ackermann_mux -> /ackermann_cmd

Autonomous driving:
  carkit_control_center -> /ackermann_cmd
```

Do not run both final publishers on `/ackermann_cmd` at the same time. For AV
driving, start `carkit_human_control` with
`vehicle_command_topic:=/ackermann_mux_unused`.
