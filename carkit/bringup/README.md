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
- command mux
- RViz

## Control Bringup Options

CARKit provides three control bringups so classes can choose between the CARKit/ADA control nodes, the CARKit command mux only, or an external F1TENTH controller stack.

### CARKit ADA Control Bringup

Use this when you want the CARKit/ADA control code to run the path tracker, command mux, stop sign behavior, and optional demo nodes.

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
- `/joy_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/purepursuit_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/emergency_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/stopsign_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/cmd_vel` (`geometry_msgs/Twist`) when `start_cmd_vel_bridge:=true`

Output:

- `/ackermann_cmd` by default, or `vehicle_command_topic` (`ackermann_msgs/AckermannDriveStamped`)

Dependencies:

- `carkit_pure_pursuit`
- `carkit_command_mux`
- `carkit_behaviors`
- `carkit_tools`

Test:

```bash
ros2 launch carkit_bringup carkit_ada_control.launch.py --show-args
ros2 topic echo /ackermann_cmd --once
```

### Command Mux Bringup

Use this when you only need CARKit command arbitration, without path tracking or external vehicle bringup.

```bash
ros2 launch carkit_bringup control_mux.launch.py
```

For `/cmd_vel` teleop tests:

```bash
ros2 launch carkit_bringup control_mux.launch.py start_cmd_vel_bridge:=true
```

Inputs:

- `/joy_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/purepursuit_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/emergency_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/stopsign_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/cmd_vel` (`geometry_msgs/Twist`) when `start_cmd_vel_bridge:=true`

Output:

- `/ackermann_cmd` by default, or `vehicle_command_topic` (`ackermann_msgs/AckermannDriveStamped`)

Test:

```bash
ros2 launch carkit_bringup control_mux.launch.py --show-args
ros2 topic echo /ackermann_cmd --once
```

### F1TENTH Controller Bringup

Use this only when your vehicle already has an external F1TENTH ROS 2 low-level controller package, such as `f1tenth_stack`, installed in this workspace or sourced from another overlay. CARKit does not ship `f1tenth_stack`.

```bash
ros2 launch carkit_bringup f1tenth_control.launch.py
```

Common arguments:

```bash
ros2 launch carkit_bringup f1tenth_control.launch.py \
  f1tenth_package:=f1tenth_stack \
  f1tenth_launch:=bringup_launch.py \
  vehicle_command_topic:=/drive
```

Optional CARKit command sources:

```bash
ros2 launch carkit_bringup f1tenth_control.launch.py \
  start_carkit_mux:=true \
  start_cmd_vel_bridge:=true \
  vehicle_command_topic:=/drive
```

Inputs when `start_carkit_mux:=true`:

- `/joy_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/purepursuit_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/emergency_cmd` (`ackermann_msgs/AckermannDriveStamped`)
- `/stopsign_cmd` (`ackermann_msgs/AckermannDriveStamped`)

Output:

- `/drive` by default, or `vehicle_command_topic` (`ackermann_msgs/AckermannDriveStamped`)

Dependencies:

- External F1TENTH package named by `f1tenth_package`
- Optional `carkit_command_mux`
- Optional `carkit_tools`

Test:

```bash
ros2 launch carkit_bringup f1tenth_control.launch.py --show-args
ros2 topic echo /drive --once
```
