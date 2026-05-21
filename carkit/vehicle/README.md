# Vehicle

Vehicle contains the vendored F1TENTH/ADA VESC stack used by CARKit human and autonomous control launches.

Packages:

- `f1tenth_system`: vendored F1TENTH controller, Ackermann mux, teleop, and VESC packages imported from `thecarlab/ada_system`.

CARKit standard vehicle command topic:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)

The F1TENTH mux accepts autonomy commands on `/drive` and controller commands through joystick teleop. The final command to the VESC bridge is `/ackermann_cmd`.

## 1. Controller

Use this when driving with the physical gamepad/controller. This launch starts the vendored F1TENTH stack:

- `joy_node`
- `joy_teleop`
- `ackermann_mux`
- `carkit_pure_pursuit` by default, for autonomous handoff
- `ackermann_to_vesc_node`
- `vesc_to_odom_node`
- `vesc_driver_node`

```bash
ros2 launch carkit_human_control controller.launch.py
```

Inputs:

- gamepad device through `joy_node`
- `/drive` (`ackermann_msgs/AckermannDriveStamped`) for autonomous command input to the F1TENTH mux
- `/purepursuit_cmd`, `/emergency_cmd`, `/stopsign_cmd` through the F1TENTH mux when those nodes are running

PS4 controller mode:

- Press `L1` once to latch manual controller mode.
- Press `L1` again to return to autonomous mode through pure pursuit.

Outputs:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/odom` (`nav_msgs/Odometry`)
- VESC serial commands through the configured VESC device

Important parameters:

- `joy_config`: defaults to `f1tenth_stack/config/joy_teleop.yaml`
- `vesc_config`: defaults to `f1tenth_stack/config/vesc.yaml`
- `mux_config`: defaults to `f1tenth_stack/config/mux.yaml`
- `vehicle_command_topic`: defaults to `/ackermann_cmd`
- `start_av_stack`: defaults to `true`
- `waypoints_file`: defaults to `carkit_bringup/waypoints/waypoints.yaml`

## 2. Keyboard

Use this when you want simple keyboard control without the controller stack. This launch starts the VESC bridge/driver and CARKit's keyboard Ackermann publisher.

```bash
ros2 launch carkit_human_control keyboard.launch.py
```

Keyboard controls:

- `w` / `s`: increase or decrease speed
- `a` / `d`: steer left or right
- `x`: center steering
- `space`: stop
- `q`: quit

Outputs:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/odom` (`nav_msgs/Odometry`)
- VESC serial commands through the configured VESC device

Important parameters:

- `vehicle_command_topic`: defaults to `/ackermann_cmd`
- `vesc_config`: defaults to `f1tenth_stack/config/vesc.yaml`

## Test

After building inside Docker:

```bash
source install/setup.bash
ros2 launch carkit_human_control controller.launch.py --show-args
ros2 launch carkit_human_control keyboard.launch.py --show-args
ros2 topic echo /ackermann_cmd --once
```

Before driving, confirm the VESC serial device in `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml` matches the hardware visible inside Docker.
