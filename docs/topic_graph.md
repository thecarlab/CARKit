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
/joy_cmd + /purepursuit_cmd + /emergency_cmd + /stopsign_cmd -> carkit_command_mux -> /ackermann_cmd
```

Vehicle command modes:

```text
controller_only.launch.py:
  /joy_cmd -> carkit_command_mux -> /ackermann_cmd or /drive

ackermann_input.launch.py:
  autonomy /ackermann_cmd -> optional carkit_vehicle_control relay -> /ackermann_cmd or /drive
  keyboard_ackermann -> /ackermann_cmd -> optional relay -> /drive
```

Control bringup options:

```text
f1tenth_control.launch.py:
  external F1TENTH bringup + optional CARKit mux/bridge -> /drive

carkit_ada_control.launch.py:
  CARKit pure pursuit + behaviors + command mux + optional ADA demos -> /ackermann_cmd
```

Mapping flow:

```text
/cloud_in -> carkit_scanmatcher -> /current_pose, /map, /map_array, /path
/map_array -> carkit_graph_based_slam -> /modified_map, /modified_path
```
