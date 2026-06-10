# TODO
- [ ] Need two new folder: carkit_behaviors (mingyu) and carkit_control_center (Ren)
- [ ] Main work for carkit_control_center is to link Mingyu's behaviors local control results, with Tianyang's navigation global control results. (Ren)

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

The mux also accepts Nav2 commands on `/drive`. Joystick commands on `/teleop`
have higher priority than navigation commands.

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

## Verify

```bash
ros2 topic echo /joy --once
ros2 topic echo /teleop --once
ros2 topic echo /ackermann_cmd --once
ros2 topic echo /odom --once
```
