# Topic Graph Overview

Nav2 mapping:

```text
sllidar_ros2 -> /scan
VESC odometry -> /odom
/scan + /odom + base_link->laser TF -> SLAM Toolbox -> /map
/map -> nav2_map_server map_saver_cli -> /workspaces/CARKit/map/*.yaml + *.pgm
```

Nav2 localization and planning:

```text
/scan + /odom + /initialpose + /workspaces/CARKit/map/<map>.yaml -> AMCL/Nav2
AMCL -> map->odom TF + /amcl_pose
Nav2 planner/controller -> /cmd_vel
/cmd_vel -> carkit_amcl twist_to_ackermann -> /drive
/drive -> ackermann_mux -> /ackermann_cmd
/ackermann_cmd -> vesc_ackermann -> vesc_driver -> vehicle
```

Traffic control:

```text
RealSense color + aligned depth + camera_info
  -> carkit_perception
  -> /yolo/detections_3d
  -> carkit_behavior
  -> /behavior
  -> ackermann_mux
  -> /ackermann_cmd
```

Ackermann mux priorities are Nav2 `/drive` `10`, behavior `/behavior` `50`,
and joystick `/teleop` `100`.
