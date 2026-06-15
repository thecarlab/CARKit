# Vehicle

The vendored F1TENTH/VESC stack provides the joystick, command mux, vehicle
driver, and odometry used by CARKit.

Launch joystick control:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Topic flow:

```text
joystick -> /joy -> joy_teleop -> /teleop
/teleop or Nav2 /drive -> ackermann_mux -> /ackermann_cmd
/ackermann_cmd -> VESC
VESC feedback -> /odom
```

See [../control/README.md](../control/README.md) for launch arguments and
tuning instructions.
