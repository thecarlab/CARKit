# Topic Graph Overview

Sensor flow:

```text
sllidar_ros2 /scan -> carkit_sensor_transforms -> /cloud_in
realsense2_camera /camera/camera/color/image_raw -> carkit_perception -> /yolo/detections
realsense2_camera /camera/camera/imu -> carkit_sensor_transforms -> /imu_transformed
```

Autonomy flow:

```text
/cloud_in + map.pcd -> carkit_lidar_localization -> /pcl_pose
/pcl_pose + /follow_path -> carkit_pure_pursuit -> /purepursuit_cmd
/yolo/detections -> carkit_behaviors -> /stopsign_cmd
/teleop + /drive + /purepursuit_cmd + /emergency_cmd + /stopsign_cmd -> ackermann_mux -> /ackermann_cmd
```

Vehicle command modes:

```text
controller.launch.py:
  gamepad -> joy_node -> joy_teleop -> ackermann_mux -> /ackermann_cmd
  CARKit autonomy command topics -> ackermann_mux -> /ackermann_cmd
  /ackermann_cmd -> vesc_ackermann -> vesc_driver -> vehicle

keyboard.launch.py:
  keyboard_ackermann -> /ackermann_cmd -> vesc_ackermann -> vesc_driver -> vehicle
```

Control bringup options:

```text
carkit_ada_control.launch.py:
  CARKit pure pursuit + behaviors + ackermann_mux + optional ADA demos -> /ackermann_cmd

carkit_nav2_av.launch.py mode:=mapping:
  /scan + /odom + base_link->laser TF -> slam_toolbox -> /map
  nav2_map_server map_saver_cli -> map.yaml + map.pgm

carkit_nav2_av.launch.py mode:=navigation:
  /scan + saved map.yaml + /initialpose + /odom -> AMCL/Nav2
  Nav2 /cmd_vel -> carkit_navigation twist_to_ackermann -> /drive
  /drive + /stopsign_cmd -> ackermann_mux -> /ackermann_cmd
```

Mapping flow:

```text
/cloud_in -> carkit_scanmatcher -> /current_pose, /map, /map_array, /path
/map_array -> carkit_graph_based_slam -> /modified_map, /modified_path
```
