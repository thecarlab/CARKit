# CARKit Control

This folder contains the nodes that decide which driving command reaches the
vehicle.

- `carkit_human_control`: launches joystick teleop and the F1TENTH/VESC vehicle
  stack.
- `carkit_control_center`: chooses the final command source and publishes
  `/ackermann_cmd`.
- `carkit_behavior`: watches perception and navigation context, then publishes
  stop overrides for road behaviors.

## Topic Flow

Manual driving:

```text
/joy
  -> joy_teleop
  -> /teleop
  -> ackermann_mux
  -> /ackermann_cmd
  -> ackermann_to_vesc_node
  -> VESC motor/servo commands
```

Autonomous driving:

```text
Nav2 /cmd_vel
  -> twist_to_ackermann
  -> /drive

/yolo/detections_2d + /scan + /plan + /odom
  -> behavior_center_node
  -> /behavior/override_active
  -> /behavior/override_cmd
  -> /behavior/state

/teleop + /drive + /behavior/* + /enable_autonomous_control
  -> control_center_node
  -> /ackermann_cmd
  -> ackermann_to_vesc_node
  -> VESC motor/servo commands
```

When autonomous mode is enabled, `control_center_node` uses this priority:

1. `EMERGENCY_STOP`: publish zero speed.
2. `HUMAN_CONTROL`: publish fresh `/teleop`, otherwise zero speed.
3. `AUTO_DRIVE`: publish fresh behavior override if active, otherwise fresh
   `/drive`, otherwise zero speed.

Launch autonomous driving with the old mux output moved away from
`/ackermann_cmd`:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/ackermann_mux_unused
```

## Control Center Topics

Subscriptions:

- `/joy`
- `/teleop`
- `/drive`
- `/behavior/override_active`
- `/behavior/override_cmd`
- `/enable_autonomous_control`

Publications:

- `/ackermann_cmd`
- `/control_center/main_state`
- `/control_center/selected_cmd`
- `/control_center/debug`

Main tuning file:

- `carkit_control_center/config/control_center.yaml`

Useful parameters:

- `publish_rate_hz`: final command publish rate.
- `auto_button`: joystick button for `AUTO_DRIVE` when button mode is used.
- `human_button`: joystick button for `HUMAN_CONTROL` when button mode is used.
- `estop_button`: joystick button for `EMERGENCY_STOP`.
- `clear_estop_button`: joystick button to clear `EMERGENCY_STOP`.
- `teleop_timeout_sec`: max age for manual command.
- `nav2_timeout_sec`: max age for Nav2 command.
- `behavior_timeout_sec`: max age for behavior override.
- `max_speed`: final speed clamp.
- `max_steering_angle`: final steering clamp.
- `initial_state`: startup mode.
- `use_autonomy_enable_topic`: use `/enable_autonomous_control` instead of
  joystick mode buttons.
- `autonomy_enable_topic`: topic name for autonomous enable messages.

## Behavior Center Topics

Subscriptions:

- `/control_center/main_state`
- `/yolo/detections_2d`
- `/scan`
- `/odom`
- `/plan`
- `/camera/camera/color/camera_info`

Publications:

- `/behavior/state`
- `/behavior/override_active`
- `/behavior/override_cmd`
- `/behavior/stop_sign_position`
- `/behavior/traffic_light_position`
- `/behavior/stop_sign_markers` (`visualization_msgs/MarkerArray`)
- `/behavior/traffic_light_markers` (`visualization_msgs/MarkerArray`)

The `*_markers` topics are the Foxglove/RViz visualization topics. The
`*_position` topics remain available as machine-readable `PointStamped`
outputs.

Behavior only affects the car while `/control_center/main_state` is
`AUTO_DRIVE`. Outside autonomous mode it publishes inactive behavior state.

Priority order:

1. Stop sign stop override.
2. Traffic light stop override.
3. Normal Nav2 driving.

## Stop Sign Logic

Stop signs use YOLO plus lidar plus the Nav2 global plan.

1. Read stop-sign detections from `/yolo/detections_2d`.
2. Reject detections below `stop_sign_min_confidence`.
3. Use the detection bearing and `/scan` to estimate distance.
4. Transform the sign position into the map frame.
5. Merge repeated detections into a stable track.
6. Project the sign and robot onto `/plan`.
7. Stop once when the robot reaches the configured distance before the sign.
8. Hold zero speed for `stop_sign_stop_duration_sec`.
9. Do not stop for the same sign again unless a new goal/path rearms it.

Stop sign parameters in `carkit_behavior/config/behavior_center.yaml`:

- `stop_sign_min_confidence`
- `stop_sign_required_observations`
- `stop_sign_stop_before_distance_m`
- `stop_sign_stop_line_tolerance_m`
- `stop_sign_stop_duration_sec`
- `stop_sign_cooldown_sec`
- `stop_sign_rearm_distance_m`
- `stop_sign_lidar_angle_window_deg`
- `stop_sign_lidar_min_range_m`
- `stop_sign_lidar_max_range_m`
- `stop_sign_track_match_distance_m`
- `stop_sign_clear_distance_m`
- `stop_sign_map_frame`
- `robot_base_frame`
- `plan_goal_change_distance_m`

## Traffic Light Logic

Traffic lights use the classified traffic-light output in
`/yolo/detections_2d.traffic_lights`.

1. Read each traffic light's `traffic_light_color`.
2. Reject detections below `traffic_light_min_confidence`.
3. Use lidar and map transform to track the light position.
4. Project the light onto the Nav2 path.
5. If the light is red or yellow, close enough, and confirmed for enough
   frames, publish a zero-speed override.
6. If the light is green for enough frames, release the override and continue
   normal Nav2 driving.

The behavior node compares `traffic_light_color` to
`YoloTrafficLightDetection2D` constants:

- `1`: red
- `2`: yellow
- `3`: green

Traffic light parameters:

- `traffic_light_min_confidence`
- `traffic_light_required_observations`
- `traffic_light_stop_required_frames`
- `traffic_light_green_required_frames`
- `traffic_light_stop_ahead_distance_m`
- `traffic_light_lidar_angle_window_deg`
- `traffic_light_lidar_min_range_m`
- `traffic_light_lidar_max_range_m`
- `traffic_light_track_match_distance_m`
- `traffic_light_clear_distance_m`

## Shared Sensor Parameters

- `detection_timeout_sec`: max age for YOLO detections.
- `scan_topic`: lidar scan topic.
- `odom_topic`: odometry topic.
- `global_plan_topic`: Nav2 global plan topic.
- `scan_timeout_sec`: max age for lidar scans.
- `camera_info_topic`: camera intrinsics topic.
- `camera_horizontal_fov_deg`: fallback FOV if intrinsics are missing.
- `camera_to_scan_yaw_offset_rad`: yaw offset from camera to lidar.
- `camera_forward_offset_m`: camera forward offset from lidar.
- `camera_lateral_offset_m`: camera lateral offset from lidar.
