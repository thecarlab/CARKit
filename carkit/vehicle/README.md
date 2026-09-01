# Vehicle adapters

CARKit supports one installed vehicle adapter at a time:

- `osracer`: serial chassis, LakiBeam lidar, and UVC camera.
- `f1tenth`: VESC chassis, URG lidar, and RealSense camera.

Select it in the WebUI or run `docker/install_carkit.sh osracer|f1tenth`.
Both adapters sit behind `carkit_bringup` and the same `/odom`, `/scan`,
`/camera/...`, `/teleop`, and `/ackermann_cmd` contracts. Student algorithms
must not import adapter packages.

The integrated OSRacer stack provides joystick input, the serial chassis
driver, odometry, IMU, battery state, LakiBeam lidar, UVC camera, and
CARKit-compatible command routing. F1TENTH sources are fetched only when that
adapter is installed.

Manual driving and mapping use the OSRacer command relay directly.
Autonomous driving uses `carkit_control_center` as the final `/ackermann_cmd`
publisher.

## Launch

For manual driving, mapping, and vehicle checks:

```bash
ros2 launch carkit_human_control joystick.launch.py
ros2 launch osracer_bringup sensors_launch.py
```

For autonomous driving, remap the manual OSRacer relay away from `/ackermann_cmd`
and start the control center in another terminal:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/manual_command_unused
```

```bash
ros2 launch carkit_control_center control_center.launch.py
```

## Topic Flow

```text
joystick -> /joy -> osracer joystick teleop -> /teleop
/teleop -> osracer command relay -> /ackermann_cmd # manual/mapping
/teleop + /drive + /behavior/* + /joy -> carkit_control_center -> /ackermann_cmd
/ackermann_cmd -> osracer_base -> chassis
chassis feedback -> osracer_base -> /odom + /imu/data + /battery_state
```

The control-center path is for autonomous driving. Direct human control keeps
the default `vehicle_command_topic:=/ackermann_cmd`.

## Configuration

- Joystick config:
  `carkit/vehicle/osracer/osracer_bringup/config/joy_teleop.yaml`
- Chassis driver:
  `carkit/vehicle/osracer/osracer_base/osracer_base/chassis_driver.py`
- Udev rule:
  `carkit/vehicle/osracer/osracer_base/udev/99-osrbot-osracer.rules`

See [../control/README.md](../control/README.md) for control-center states and
the autonomous command workflow.
