# Joystick Control

Package: `carkit_human_control`

CARKit human control uses a joystick to publish Ackermann commands through the
F1TENTH mux and VESC vehicle stack.

## Launch

Connect the joystick and VESC, then run:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Show all launch arguments:

```bash
ros2 launch carkit_human_control joystick.launch.py --show-args
```

## Topic Flow

```text
/dev/input/js0 -> joy_node -> /joy
/joy -> joy_teleop -> /teleop
/teleop -> ackermann_mux -> /ackermann_cmd
/ackermann_cmd -> ackermann_to_vesc_node -> VESC motor and servo commands
VESC feedback -> vesc_to_odom_node -> /odom
```

The mux accepts Nav2 commands on `/drive` and road-rule stop overrides on
`/behavior`. Priorities are navigation `10`, behavior `50`, and joystick `100`,
so a human can always override an automated stop.

## Common Arguments

- `joy_config`: joystick device, axis mapping, deadzone, speed, and steering
  configuration. Defaults to
  `carkit/vehicle/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`.
- `vesc_config`: VESC port, calibration, limits, wheelbase, and odometry
  configuration. Defaults to
  `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`.
- `mux_config`: command topics, priorities, and timeouts. Defaults to
  `carkit/vehicle/f1tenth_system/f1tenth_stack/config/mux.yaml`.
- `vehicle_command_topic`: final Ackermann command topic consumed by the
  vehicle controller. Defaults to `/ackermann_cmd`.

Example with custom configurations:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  joy_config:=/path/to/joy_teleop.yaml \
  vesc_config:=/path/to/vesc.yaml
```

## Common Tuning

In `joy_teleop.yaml`:

- `joy.ros__parameters.dev`: joystick device, normally `/dev/input/js0`
- `deadzone`: ignores small joystick movement
- `autorepeat_rate`: joystick command publishing rate
- `mode_toggle_button`: button that switches between joystick `/teleop` and
  Nav2 `/drive` control
- `manual_mode_initial`: whether joystick control is active at startup
- `drive-speed.scale`: maximum commanded speed
- `drive-steering_angle.scale`: maximum commanded steering angle

In `vesc.yaml`:

- `port`: VESC serial device, normally `/dev/ttyACM0`
- `speed_to_erpm_gain`: converts vehicle speed to motor ERPM
- `steering_angle_to_servo_gain`: steering angle calibration
- `steering_angle_to_servo_offset`: centered steering calibration
- `servo_min` and `servo_max`: steering servo limits
- `wheelbase`: wheelbase used for odometry

In `mux.yaml`:

- `priority`: higher values take control over lower values
- `timeout`: time before an inactive command source is ignored

## Road-Rule Behavior

Package: `carkit_behavior`

After starting the RealSense camera and vehicle/Nav2 bring-up:

```bash
ros2 launch carkit_behavior road_rules.launch.py
```

The behavior node consumes typed detections from `/yolo/detections_3d`.

- A qualifying red light immediately latches a stop.
- A qualifying green light or manual reset releases the red-light stop.
- Yellow is reported without stopping.
- Stop signs and traffic lights use the fixed `stop_distance` trigger.
- A stop sign immediately requests a stop, waits for odometry speed below
  `0.05 m/s`, holds for three seconds, and enters a five-second cooldown.
- Detections without a valid 3D position do not trigger a stop.
- The behavior node publishes only zero-speed commands. It never starts motion.

Normal topic flow:

```text
/yolo/detections_3d + /odom + /drive -> carkit_behavior -> /behavior
/drive or /behavior or /teleop -> ackermann_mux -> /ackermann_cmd
```

Inspect or reset the behavior:

```bash
ros2 topic echo /carkit_behavior/state
ros2 service call /carkit_behavior/reset std_srvs/srv/Trigger
```

Common behavior parameters:

- `traffic_light_min_confidence`
- `stop_sign_min_confidence`
- `max_lateral_offset`
- `stop_distance`
- `stop_speed_threshold`
- `stop_hold_seconds`
- `stop_cooldown_seconds`
- `stop_rearm_absence_seconds`

## Verify

```bash
ros2 topic echo /joy --once
ros2 topic echo /teleop --once
ros2 topic echo /ackermann_cmd --once
ros2 topic echo /odom --once
```
