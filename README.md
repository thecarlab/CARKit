# CARKit

CARKit is a ROS 2 Humble education stack for small Ackermann autonomous
vehicles. The supported autonomous workflow uses Nav2 for SLAM Toolbox
mapping, AMCL localization, path planning, obstacle avoidance, and vehicle
commands.

## Supported Platform

- Jetson Orin Nano with JetPack 6.x / L4T 36.x
- Docker with NVIDIA runtime
- Slamtec SLLiDAR
- Intel RealSense camera
- Ackermann/F1TENTH-style vehicle with VESC odometry

ROS 2 and CARKit dependencies run inside `ariiees/carkit:latest`. The host
only needs JetPack, Docker, Git, display access for RViz, and device access.

## Setup

On the Jetson host:

```bash
git clone https://github.com/thecarlab/CARKit.git
cd CARKit
./carkit/setup_vendor_repos.sh
docker pull ariiees/carkit:latest
./docker/run_jetson.sh
```

Inside the container:

```bash
./docker/build_workspace.sh
source install/setup.bash
```

The repository is mounted at `/workspaces/CARKit` in the container.

## Nav2 Mapping

Start VESC odometry:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

In a second terminal, start SLAM Toolbox:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=mapping \
  start_static_tf:=false
```

Drive through the environment, then save the occupancy map from a third
terminal:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspaces/CARKit/map/map
```

All generated and included maps belong in the repository's `map/` folder.

## Nav2 Navigation

Start VESC odometry:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

In a second terminal, launch Nav2 with a saved map:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  start_static_tf:=false \
  map:=/workspaces/CARKit/map/map.yaml
```

In RViz, set **2D Pose Estimate**, wait for AMCL to converge, then send a
**Nav2 Goal**.

The included example map can be selected with:

```bash
map:=/workspaces/CARKit/map/map_3f.yaml
```

See [carkit/navigation/README.md](carkit/navigation/README.md) for launch
arguments, topic flow, and troubleshooting.

## Repository Layout

```text
carkit/
  navigation/    Nav2 localization, mapping, and planning
  sensors/       sensor drivers and transform nodes
  perception/    camera perception
  control/       joystick control launch and instructions
  vehicle/       F1TENTH/VESC vehicle stack
  tools/         classroom utilities
map/             all Nav2 occupancy maps
docker/          Docker image and workspace scripts
docs/            topic graph and troubleshooting
```

## Verify

Inside Docker after building:

```bash
ros2 launch carkit_navigation navigation.launch.py --show-args
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

## Troubleshooting

See [docs/troubleshooting.md](docs/troubleshooting.md).

## Contributing

- Keep all Nav2 packages under `carkit/navigation/`.
- Keep all occupancy maps under `map/`; do not add package-local map folders.
- Do not commit build outputs, Docker credentials, tokens, or machine-specific
  paths.
- Update `carkit/navigation/README.md` when changing Nav2 topics, parameters,
  launch files, or dependencies.
