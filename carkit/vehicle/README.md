# Vehicle

The vendored F1TENTH/VESC stack provides joystick input, low-level VESC
drivers, odometry, and legacy Ackermann mux packages used by CARKit.

Manual driving and mapping use the vendored `ackermann_mux` path directly.
Autonomous driving uses `carkit_control_center` as the final `/ackermann_cmd`
publisher.

## Launch

For manual driving, mapping, and vehicle checks:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

For autonomous driving, remap the legacy mux output away from `/ackermann_cmd`
and start the control center in another terminal:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/ackermann_mux_unused
```

```bash
ros2 launch carkit_control_center control_center.launch.py
```

## Topic Flow

```text
joystick -> /joy -> joy_teleop -> /teleop
/teleop -> ackermann_mux -> /ackermann_cmd        # manual/mapping
/teleop + /drive + /behavior/* + /joy -> carkit_control_center -> /ackermann_cmd
/ackermann_cmd -> ackermann_to_vesc_node -> VESC
VESC feedback -> vesc_to_odom_node -> /odom
```

The control-center path is for autonomous driving. Direct human control keeps
the default `vehicle_command_topic:=/ackermann_cmd`.

## Configuration

- Joystick config:
  `carkit/vehicle/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml`
- VESC config:
  `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`
- Legacy mux config:
  `carkit/vehicle/f1tenth_system/f1tenth_stack/config/mux.yaml`

See [../control/README.md](../control/README.md) for control-center states and
the autonomous command workflow.
