# Bringup

Package: `carkit_bringup`

Bringup contains full-stack launch files, maps, waypoints, and RViz configs.

## Launch Full Stack

```bash
ros2 launch carkit_bringup carkit.launch.py
```

With custom files:

```bash
ros2 launch carkit_bringup carkit.launch.py \
  rviz_config:=/path/to/config.rviz \
  waypoints_file:=/path/to/waypoints.yaml
```

## Test

Check launch arguments without starting hardware:

```bash
ros2 launch carkit_bringup carkit.launch.py --show-args
```

After launch:

```bash
ros2 topic list
ros2 topic echo /ackermann_cmd --once
```

Full bringup starts:

- SLLiDAR
- RealSense
- sensor transforms
- LiDAR localization
- pure pursuit
- stop sign behavior
- F1TENTH Ackermann mux
- RViz

## Control Bringup

Use CARKit/ADA control when you want the autonomy stack to run the path tracker, F1TENTH Ackermann mux, stop sign behavior, and optional demo nodes.

For physical vehicle control, use the two vehicle launch files:

```bash
ros2 launch carkit_human_control controller.launch.py
ros2 launch carkit_human_control keyboard.launch.py
```

### CARKit ADA Control

```bash
ros2 launch carkit_bringup carkit_ada_control.launch.py
```

With a custom waypoint file and vehicle command topic:

```bash
ros2 launch carkit_bringup carkit_ada_control.launch.py \
  waypoints_file:=/path/to/waypoints.yaml \
  vehicle_command_topic:=/ackermann_cmd
```

Optional demo/test modes:

```bash
ros2 launch carkit_bringup carkit_ada_control.launch.py ada_demo:=demo1
ros2 launch carkit_bringup carkit_ada_control.launch.py ada_demo:=demo2
ros2 launch carkit_bringup carkit_ada_control.launch.py start_cmd_vel_bridge:=true
```

Inputs:

- `/pcl_pose` (`geometry_msgs/PoseStamped`)
- `/follow_path` (`nav_msgs/Path`)
- `/enable_autonomous_control` (`std_msgs/Int8`)
- `/reverse_mode` (`std_msgs/Int8`)
- `/purepursuit_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/emergency_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/stopsign_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/cmd_vel` (`geometry_msgs/Twist`) when `start_cmd_vel_bridge:=true`

Output:

- `/ackermann_cmd` by default, or `vehicle_command_topic` (`ackermann_msgs/AckermannDriveStamped`)

Dependencies:

- `ackermann_mux`
- `carkit_pure_pursuit`
- `carkit_behaviors`
- `carkit_tools`

Test:

```bash
ros2 launch carkit_bringup carkit_ada_control.launch.py --show-args
ros2 topic echo /ackermann_cmd --once
```

## Nav2 AV Bringup

Use the Nav2 AV workflow when you want RViz initial pose, RViz goal pose,
2D SLAM/maps, Nav2 obstacle avoidance, and Ackermann output through the
existing CARKit mux.

Create a 2D map directly with SLAM Toolbox:

```bash
ros2 launch carkit_bringup carkit_nav2_av.launch.py mode:=mapping
```

On the physical car, SLAM and Nav2 also need `/odom`. Start the existing
controller/VESC launch for odometry, then disable the duplicate laser TF in
the Nav2 mapping launch:

```bash
ros2 launch carkit_human_control controller.launch.py start_av_stack:=false
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=mapping \
  start_static_tf:=false
```

Save the map from another terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f /workspaces/CARKit/carkit/planning/carkit_navigation/maps/map
```

Navigate with the saved map:

```bash
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=navigation \
  map:=/workspaces/CARKit/carkit/planning/carkit_navigation/maps/map.yaml
```

If you also start the existing controller/VESC launch for real vehicle
odometry and low-level drive output, avoid duplicate mux and static TF nodes:

```bash
ros2 launch carkit_human_control controller.launch.py start_av_stack:=false
ros2 launch carkit_bringup carkit_nav2_av.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  start_static_tf:=false \
  map:=/workspaces/CARKit/carkit/planning/carkit_navigation/maps/map.yaml
```

Nav2 inputs:

- `/scan` (`sensor_msgs/LaserScan`)
- `/odom` (`nav_msgs/Odometry`) for `odom -> base_link`
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
- `/goal_pose` or Nav2 RViz goal action

Nav2 outputs:

- `/map` (`nav_msgs/OccupancyGrid`) in mapping mode
- `/cmd_vel` (`geometry_msgs/Twist`)
- `/drive` (`ackermann_msgs/AckermannDriveStamped`)
- `/ackermann_cmd` through the existing mux when `start_command_mux:=true`
