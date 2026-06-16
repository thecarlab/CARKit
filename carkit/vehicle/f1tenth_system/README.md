# f1tenth_system

This folder vendors the F1TENTH/VESC packages that CARKit builds inside the
single ROS 2 Humble Docker image `ariiees/carkit:latest`.

Do not use old standalone F1TENTH/Foxy container workflows for CARKit bringup.
Use the CARKit launch wrappers instead.

## Active CARKit Launch

For manual driving, mapping, and vehicle checks:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

For autonomous driving, remap the legacy mux output and start the control
center:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/ackermann_mux_unused
```

```bash
ros2 launch carkit_control_center control_center.launch.py
```

## Vendored Packages

- `f1tenth_stack`: bringup launch files and configs
- `ackermann_mux`: legacy Ackermann command multiplexer
- `teleop_tools`: joystick teleop packages
- `vesc`: VESC driver, messages, and Ackermann conversion packages

CARKit uses `ackermann_mux` directly for manual driving and mapping.
Autonomous driving uses `carkit_control_center` as the final publisher of
`/ackermann_cmd`.

## Active Topic Flow

```text
/joy -> joy_teleop -> /teleop
/teleop -> ackermann_mux -> /ackermann_cmd        # manual/mapping
/teleop + /drive + /behavior/* + /joy -> carkit_control_center -> /ackermann_cmd
/ackermann_cmd -> ackermann_to_vesc_node -> VESC
VESC feedback -> vesc_to_odom_node -> /odom
```

See [../README.md](../README.md) and
[../../control/README.md](../../control/README.md) for the current CARKit
vehicle and control workflow.
