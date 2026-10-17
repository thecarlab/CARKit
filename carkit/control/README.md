# CARKit Control

This folder contains the nodes that decide which driving command reaches the
vehicle.

- `carkit_human_control`: hardware-independent optional joystick input and a
  watchdog-protected standalone relay.
- `carkit_control_center`: native C++ safety arbiter that chooses the final
  command source and publishes `/ackermann_cmd` at 30 Hz.

Behavior decisions now live in `carkit/planning/carkit_behavior`. This control
module only consumes the stable `/behavior/*` decision interface.

## Default input and command flow

The integrated launch uses keyboard control in the WebUI by default for all
chassis. Hold WASD for motion, release to stop, use Caps Lock to toggle autonomy,
and Space or the Emergency stop button to latch a stop. Reset to manual clears
it without motion. Typing in editors/terminals is excluded. Select
`manual_input:=joystick` for the common joystick adapter instead.

```text
WebUI WASD -> native bridge -> /teleop ─┐
Optional joystick -> /teleop ──────────┤
Nav2 /cmd_vel -> Twist adapter -> /drive├-> control_center -> /ackermann_cmd -> hardware
Behavior override and E-stop ─────────┘
```

The arbiter remains present in manual-only integrated sessions. OSRacer's direct
`/cmd_vel` input is disabled by default; vendor VESC joystick/multiplexer launches
are not used. An active behavior override that loses its command or heartbeat
holds zero until explicitly released. Freshness uses a monotonic clock.

Use the integrated launch for either input method:

```bash
ros2 launch carkit_bringup carkit.launch.py chassis:=osracer profile:=reference
# Optional joystick:
ros2 launch carkit_bringup carkit.launch.py chassis:=f1tenth manual_input:=joystick
```

The legacy `carkit_human_control joystick.launch.py` remains an OSRacer-only
standalone diagnostic launch. Do not run it alongside an integrated launch.

## Control Center Topics

Subscriptions:

- `/joy`
- `/teleop`
- `/drive`
- `/behavior/override_active`
- `/behavior/override_cmd`
- `/enable_autonomous_control`
- `/control_center/emergency_stop` (Bool: latch stop / reset to manual)

Publications:

- `/ackermann_cmd`
- `/control_center/main_state`
- `/control_center/selected_cmd`
- `/control_center/debug`

Main tuning file:

- Shared vehicle geometry and limits: `carkit_bringup/config/hardware/*.yaml`.
- Standalone arbiter defaults: `carkit_control_center/config/control_center.yaml`.

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
