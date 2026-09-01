# CARKit Control

This folder contains the nodes that decide which driving command reaches the
vehicle.

- `carkit_human_control`: launches joystick teleop and the OSRacer vehicle
  stack.
- `carkit_control_center`: native C++ safety arbiter that chooses the final
  command source and publishes `/ackermann_cmd` at 30 Hz.

Behavior decisions now live in `carkit/planning/carkit_behavior`. This control
module only consumes the stable `/behavior/*` decision interface.

## Topic Flow

Manual driving:

```text
/joy
  -> osracer joystick teleop
  -> /teleop
  -> osracer command relay
  -> /ackermann_cmd
  -> osracer_base
  -> OSRacer chassis
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
  -> osracer_base
  -> OSRacer chassis
```

When autonomous mode is enabled, `control_center_node` uses this priority:

1. `EMERGENCY_STOP`: publish zero speed.
2. `HUMAN_CONTROL`: publish fresh `/teleop`, otherwise zero speed.
3. `AUTO_DRIVE`: publish fresh behavior override if active, otherwise fresh
   `/drive`, otherwise zero speed.

Launch autonomous driving with the manual relay output moved away from
`/ackermann_cmd`:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/manual_command_unused
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
- `status_publish_rate_hz`: heartbeat rate for unchanged state and command-source text.
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

The WebUI control-authority switch publishes `std_msgs/msg/Int8` on
`/enable_autonomous_control` (`0` for Human, `1` for Autonomous). Human routes
fresh remote-controller `/teleop` messages; Autonomous routes fresh Ackermann
`/drive` messages. The switch reflects `/control_center/main_state`, so a mode
is not shown as active until the control center confirms it. OSRacer joystick
teleop also listens to the enable topic, keeping its physical mode toggle and
the WebUI selection synchronized.

## Behavior Decision Contract

The control center consumes `/behavior/override_active` and
`/behavior/override_cmd` only while it is in `AUTO_DRIVE`. Behavior-rule
implementation, priorities, perception tracking, and tuning are documented in
`carkit/planning/carkit_behavior/README.md`.
