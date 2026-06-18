# CARKit

CARKit is a ROS 2 Humble stack for small Ackermann autonomous vehicles. The
current workflow uses one Docker image, a mounted workspace, VESC odometry,
Nav2 mapping/navigation, camera perception, behavior overrides, and one final
control arbiter.

## Supported Platform

- Jetson Orin Nano with JetPack 6.x / L4T 36.x
- Docker with NVIDIA runtime
- Slamtec SLLiDAR publishing `/scan`
- Intel RealSense color camera
- Ackermann/F1TENTH-style vehicle with VESC odometry

ROS 2 and CARKit dependencies run inside `ariiees/carkit:latest`. The host
only needs JetPack, Docker, Git, display access for RViz, and device access.

## Setup

On the Jetson host:

```bash
git clone https://github.com/thecarlab/CARKit.git
cd CARKit
docker pull ariiees/carkit:latest
./docker/run_jetson.sh
```

Inside the container:

```bash
./docker/build_workspace.sh
source install/setup.bash
```

`build_workspace.sh` fetches vendored sensor repos and builds the mounted
workspace at `/workspaces/CARKit`.

## Topic Flow

Manual driving and mapping:

```text
/joy -> joy_teleop -> /teleop
/teleop -> ackermann_mux -> /ackermann_cmd
/ackermann_cmd -> ackermann_to_vesc_node -> VESC
VESC feedback -> /odom
```

Autonomous driving:

```text
/joy -> joy_teleop -> /teleop
Nav2 -> /cmd_vel -> twist_to_ackermann -> /drive
/yolo/detections_2d -> carkit_behavior -> /behavior/*
/teleop + /drive + /behavior/* + /joy -> carkit_control_center -> /ackermann_cmd
/ackermann_cmd -> ackermann_to_vesc_node -> VESC
VESC feedback -> /odom
```

For autonomous driving, `carkit_control_center` owns the final `/ackermann_cmd`.
Run Nav2 with `start_command_mux:=false`; this is the default in the current
launch files.

## Manual Driving And Mapping Control

For manual driving, mapping, and vehicle checks, launch human control directly:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

This launches joystick teleop, VESC, odometry, and the legacy mux path from
`/teleop` to `/ackermann_cmd`.

## Mapping

Start human control as shown above, then launch mapping:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=mapping
```

Drive through the environment, then save the occupancy map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspaces/CARKit/map/map
```

Maps belong in the repository's top-level `map/` folder.

## Navigation

For autonomous driving, start human control with the legacy mux output remapped
away from `/ackermann_cmd`, start the control center, then launch Nav2:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/ackermann_mux_unused
```

```bash
ros2 launch carkit_control_center control_center.launch.py
```

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  map:=/workspaces/CARKit/map/map.yaml
```

In RViz, set **2D Pose Estimate**, wait for AMCL to converge, then send a
**Nav2 Goal**. Press the configured auto joystick button to enter
`AUTO_DRIVE`; the default is button `0`.

The included example map is the default and can be selected explicitly with:

```bash
map:=/workspaces/CARKit/map/map_3f.yaml
```

## Perception And Behavior

Start the color-only RealSense driver and typed 2D YOLO perception together:

```bash
export LD_LIBRARY_PATH=/usr/local/lib/python3.10/dist-packages/nvidia/cusparselt/lib:$LD_LIBRARY_PATH
ros2 launch carkit_perception perception.launch.py
```

Start behavior overrides and cone obstacle publishing:

```bash
ros2 launch carkit_behavior behavior_center.launch.py
```

Behavior logic only affects commands while the control center is in
`AUTO_DRIVE`.

## Repository Layout

```text
carkit/
  control/       human teleop, behavior layer, autonomous command arbiter
  navigation/    SLAM Toolbox, AMCL, Nav2, Twist-to-Ackermann bridge
  perception/    color-only YOLO and typed 2D detection messages
  sensors/       sensor driver fetch notes and transform nodes
  vehicle/       vendored F1TENTH/VESC vehicle stack
  tools/         classroom utilities and demos
map/             all occupancy maps
docker/          image, run, build, and publish scripts
docs/            troubleshooting and diagrams
```

## Verify

Inside Docker after building:

```bash
ros2 launch carkit_navigation navigation.launch.py --show-args
ros2 launch carkit_control_center control_center.launch.py --show-args
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /ackermann_cmd --once
```

## More Docs

- [Control](carkit/control/README.md)
- [Navigation](carkit/navigation/README.md)
- [Perception](carkit/perception/README.md)
- [Sensors](carkit/sensors/README.md)
- [Vehicle](carkit/vehicle/README.md)
- [Docker](docker/README.md)
- [Troubleshooting](docs/troubleshooting.md)
