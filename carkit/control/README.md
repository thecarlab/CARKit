# Control

Control contains path tracking, emergency braking, and command muxing.

Packages:

- `carkit_pure_pursuit`
- `carkit_command_mux`

## Pure Pursuit

Launch:

```bash
ros2 launch carkit_pure_pursuit pure_pursuit_system.launch.py waypoints_file:=carkit/bringup/waypoints/waypoints.yaml
```

Enable autonomous control:

```bash
ros2 topic pub /enable_autonomous_control std_msgs/msg/Int8 "{data: 1}" --once
```

Test:

```bash
ros2 topic echo /follow_path --once
ros2 topic echo /purepursuit_cmd --once
```

## Command Mux

Launch:

```bash
ros2 run carkit_command_mux carkit_command_mux_node
```

Test:

```bash
ros2 topic echo /ackermann_cmd --once
```

Inputs:

- `/joy_cmd`
- `/purepursuit_cmd`
- `/emergency_cmd`
- `/stopsign_cmd`

Output:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)

## Emergency Brake

Launch:

```bash
ros2 run carkit_pure_pursuit emergency_braker
```

Inputs:

- `/cloud_in` (`sensor_msgs/PointCloud2`)
- `/odom` (`nav_msgs/Odometry`)

Output:

- `/emergency_cmd` (`ackermann_msgs/AckermannDriveStamped`)
