# Navigation

The `navigation` module contains the supported mapping and Nav2 workflow:

- `carkit_slam`: SLAM Toolbox 2D occupancy-grid mapping
- `carkit_amcl`: AMCL localization, Nav2 configuration, behavior trees, and the
  native C++ command/TF/waypoint adapters
- `carkit_navigation`: CMake-packaged mapping and navigation launch orchestration

Occupancy maps are stored in `/workspaces/CARKit/map` inside Docker, which is
the repository's top-level `map/` directory.

## Before Starting

Build and source the workspace:

```bash
cd /workspaces/CARKit
colcon build --symlink-install
source install/setup.bash
```

The chassis is always started in a separate terminal with:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Start the OSRacer sensors in another terminal:

```bash
ros2 launch osracer_bringup sensors_launch.py
```

This launch starts the joystick, OSRacer command relay and chassis driver,
odometry, IMU, battery state, and the `base_link -> laser` static transform.

## Mapping

Terminal 1, start the chassis:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Terminal 2, start mapping:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=mapping \
  start_lidar:=false
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspaces/CARKit/map/<map_name>
```

This creates `<map_name>.yaml` and `<map_name>.pgm` in
`/workspaces/CARKit/map`.

## Navigation

Terminal 1, start the chassis:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Start the OSRacer sensors in another terminal:

```bash
ros2 launch osracer_bringup sensors_launch.py
```

Terminal 2, start localization and navigation:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  map:=/workspaces/CARKit/map/map_3f.yaml \
  start_lidar:=false
```

- Replace `map_3f.yaml` with `map2.yaml` or another saved map when needed.

Open the browser interface at:

```text
http://<jetson-ip>:8080
```

In the Overview map panel:

1. Use **2D Pose Estimate** to set the initial vehicle pose.
2. Wait for the AMCL particle cloud to converge around the vehicle.
3. Use **Nav2 Goal** to send a single navigation goal.

## Multiple Poses In The WebUI

1. Choose the goal/waypoint tool in the Overview map panel.
2. Click and drag on the map for each pose, in driving order. Green numbered
   markers show the accumulated route.
3. Click **Start route**. Active poses turn blue while Nav2 executes one
   `NavigateThroughPoses` action.

**Clear poses** removes poses that have not started. **Cancel route** cancels
the active route. The waypoint adapter publishes text status on the legacy
`/foxglove/waypoints/status` topic for compatibility.

## Topic Flow

```text
Mapping:
  /scan + /odom -> SLAM Toolbox -> /map
  /map -> map_saver_cli -> /workspaces/CARKit/map/*.yaml + *.pgm

Navigation:
  /scan + /odom + saved map + /initialpose -> AMCL/Nav2
  Nav2 planner/controller -> /cmd_vel
  /cmd_vel -> twist_to_ackermann -> /drive
  /teleop -> osracer command relay -> /ackermann_cmd       # manual
  /drive + /teleop -> carkit_control_center -> /ackermann_cmd # autonomous
  /ackermann_cmd -> osracer_base -> OSRacer chassis
```

## Common Arguments

- `mode:=mapping|navigation`: selects the workflow
- `map:=/workspaces/CARKit/map/<name>.yaml`: selects the navigation map
- `start_lidar:=true|false`: starts or skips CARKit's optional SLLiDAR driver;
  use `false` when the OSRacer LakiBeam sensor launch is running
- `start_odom_tf:=true|false`: republishes `/odom` as `odom -> base_link`
- `start_command_mux:=true|false`: deprecated optional direct `/drive` relay

Show all arguments:

```bash
ros2 launch carkit_navigation navigation.launch.py --show-args
```

## Configuration

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`
- `carkit/navigation/carkit_amcl/behavior_trees/`
- `carkit/navigation/carkit_slam/config/slam_toolbox_params.yaml`
- `carkit/vehicle/osracer/osracer_bringup/launch/bringup_launch.py`
- `map/`: generated and included occupancy maps

## Verification

```bash
ros2 topic echo /map --once
ros2 topic echo /odom --once
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

If AMCL does not publish, wait for the map to load and set the initial pose in
the WebUI. If odometry distance or heading is inaccurate, validate the legacy
OSRacer `o` telemetry against measured motion before tuning Nav2.
