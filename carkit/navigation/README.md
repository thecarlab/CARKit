# Navigation

The `navigation` module contains the supported mapping and Nav2 workflow:

- `carkit_slam`: SLAM Toolbox 2D occupancy-grid mapping
- `carkit_amcl`: AMCL localization, Nav2 configuration, behavior trees, and the
  Twist-to-Ackermann bridge
- `carkit_navigation`: mapping and navigation launch orchestration

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

This launch starts the joystick, Ackermann command mux, VESC driver, odometry,
low-level vehicle controller, throttle interpolator, and the
`base_link -> laser` static transform.

## Mapping

Terminal 1, start the chassis:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

Terminal 2, start mapping:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=mapping \
  start_static_tf:=false
```

`start_static_tf:=false` prevents a duplicate `base_link -> laser` transform,
because the chassis launch already publishes it.

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

Terminal 2, start localization and navigation:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  start_static_tf:=false \
  map:=/workspaces/CARKit/map/map_3f.yaml
```

- `start_command_mux:=false` prevents starting a second Ackermann mux.
- `start_static_tf:=false` prevents publishing the laser transform twice.
- Replace `map_3f.yaml` with `map2.yaml` or another saved map when needed.

In RViz:

1. Use **2D Pose Estimate** to set the initial vehicle pose.
2. Wait for the AMCL particle cloud to converge around the vehicle.
3. Use **Nav2 Goal** to send a single navigation goal.

## Multiple Poses In RViz

To navigate through several poses:

1. In the **Navigation 2** panel, click
   **Waypoint / Nav Through Poses Mode**.
2. Select **Nav2 Goal** from the RViz toolbar. Do not use **2D Goal Pose**.
3. Click and drag on the map to add each pose and its heading. Add the poses in
   the order in which the vehicle should visit them.
4. Click **Start Nav Through Poses** to execute them as one continuous
   navigation task.

Use **Cancel Accumulation** to clear poses that have not been started.
**Start Waypoint Following** executes the same list through the waypoint
follower and may run the configured task at every waypoint.

## Topic Flow

```text
Mapping:
  /scan + /odom -> SLAM Toolbox -> /map
  /map -> map_saver_cli -> /workspaces/CARKit/map/*.yaml + *.pgm

Navigation:
  /scan + /odom + saved map + /initialpose -> AMCL/Nav2
  Nav2 planner/controller -> /cmd_vel
  /cmd_vel -> twist_to_ackermann -> /drive
  /drive + teleop commands -> ackermann_mux -> /ackermann_cmd
  /ackermann_cmd -> ackermann_to_vesc -> throttle_interpolator -> VESC
```

## Common Arguments

- `mode:=mapping|navigation`: selects the workflow
- `map:=/workspaces/CARKit/map/<name>.yaml`: selects the navigation map
- `start_lidar:=true|false`: starts or skips the SLLiDAR driver
- `start_static_tf:=true|false`: publishes or skips `base_link -> laser`
- `start_odom_tf:=true|false`: republishes `/odom` as `odom -> base_link`
- `start_command_mux:=true|false`: starts or skips the Ackermann mux
- `use_rviz:=true|false`: starts or skips RViz
- `mapping_rviz_config`: overrides the mapping RViz configuration
- `planning_rviz_config`: overrides the navigation RViz configuration

Show all arguments:

```bash
ros2 launch carkit_navigation navigation.launch.py --show-args
```

## Configuration

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`
- `carkit/navigation/carkit_amcl/behavior_trees/`
- `carkit/navigation/carkit_slam/config/slam_toolbox_params.yaml`
- `carkit/navigation/carkit_slam/rviz/mapping.rviz`
- `carkit/navigation/carkit_navigation/rviz/navigation.rviz`
- `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`
- `map/`: generated and included occupancy maps

## Verification

```bash
ros2 topic echo /map --once
ros2 topic echo /odom --once
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

If AMCL does not publish, wait for the map to load and set the initial pose in
RViz. If odometry distance or heading is inaccurate, calibrate the VESC
parameters in `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`.
