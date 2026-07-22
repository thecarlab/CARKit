# Tools

Package: `carkit_tools`

These are classroom utilities and demos. They are not part of the active
Nav2-control-center autonomy path unless launched explicitly.

## ROS 2 Node CPU Monitor

The standalone monitor combines the local ROS graph with container process
information and shows PID, CPU, RSS memory, thread count, process name, and
node-to-process mapping.

For an SSH session, use the continuously refreshed terminal view. It does not
start PyQt or require X11/remote desktop:

```bash
ssh <user>@<car-ip>
docker exec -it carkit bash
source install/setup.bash
python3 carkit/tools/node_cpu_monitor.py --terminal
```

The table refreshes once per second by default. Use a slower interval to reduce
the monitor's own overhead, or `Ctrl+C` to exit:

```bash
python3 carkit/tools/node_cpu_monitor.py --terminal --refresh-ms 2000
```

Use `--no-clear` to append samples for a terminal log. To print only one sample:

```bash
python3 carkit/tools/node_cpu_monitor.py --once
```

The original GUI remains available when a display is present:

```bash
python3 carkit/tools/node_cpu_monitor.py
```

No additional Python packages are required by the CARKit container. Terminal
mode uses `rclpy` and `psutil`; PyQt5 is imported only in GUI mode.

ROS 2 does not provide a standard node-to-PID API. Standalone nodes are mapped
from their ROS command-line remaps or executable names. Multiple nodes hosted
by one component container share a process, so their CPU cannot be separated
reliably. Terminal mode lists the combined process usage and marks graph nodes
without a unique PID as unmapped; the GUI shows the same distinction on the
**ROS Processes** tab.

## Jetson System Monitor

The standalone Jetson monitor reads Linux procfs/sysfs directly and does not
require ROS, PyQt, or a rebuild. It shows total CPU usage, each CPU's current
frequency, thermal-zone temperatures, and voltage, current, instantaneous
power, and running-average power for the INA3221 rails. `VDD_IN` is already the
total module input power; do not add the other rails to it.

Over SSH, enter the privileged CARKit container and start display-only mode:

```bash
docker exec -it carkit bash
cd /workspaces/CARKit
python3 carkit/tools/jetson_system_monitor.py mode1
```

Mode 2 displays the same table and writes one flushed CSV row per sample. By
default, files are stored under `log/system_monitor/`, which persists in the
mounted workspace and is ignored by Git:

```bash
python3 carkit/tools/jetson_system_monitor.py mode2
python3 carkit/tools/jetson_system_monitor.py mode2 --output /tmp/run.csv
```

Both modes sample once per second. The interval can be changed, and terminal
samples can be appended instead of refreshing in place:

```bash
python3 carkit/tools/jetson_system_monitor.py mode1 --interval 2.0
python3 carkit/tools/jetson_system_monitor.py mode1 --no-clear
```

Use `Ctrl+C` to stop. Empty thermal zones are shown as `offline`; missing or
unreadable sensors are shown as `unavailable` and left blank in CSV output.

## Interactive Waypoints

```bash
ros2 launch carkit_tools interactive_waypoints.launch.py
```

The node waits for `/pcl_pose`, creates an interactive marker in `map`, and
publishes:

- `/follow_path` (`nav_msgs/Path`)

Verify:

```bash
ros2 topic echo /follow_path --once
```

## Legacy Object Tracking Demo

```bash
ros2 launch carkit_tools object_tracking.launch.py target_object_type:=book
```

This launch starts:

- `object_position`, which subscribes to legacy string `/yolo/detections` and
  aligned depth, then publishes `/object_position`
- `path_tracker`, which subscribes to `/object_position` and publishes
  `/object_path` and `/object_waypoints`

Current `carkit_perception` publishes typed `/yolo/detections_2d`, not the
legacy string `/yolo/detections`, so this demo needs a compatible legacy YOLO
publisher or a future adapter before it can consume current perception output.

Verify when a compatible detection source is running:

```bash
ros2 topic echo /object_position --once
ros2 topic echo /object_path --once
ros2 topic echo /object_waypoints --once
```

## Console Scripts

```bash
ros2 run carkit_tools interactive_waypoints
ros2 run carkit_tools demo1
ros2 run carkit_tools demo2
ros2 run carkit_tools distance_metrics
ros2 run carkit_tools object_angle
ros2 run carkit_tools object_position
ros2 run carkit_tools path_tracker
ros2 run carkit_tools cmd_vel_to_ackermann
```

## Stop Latency Monitor

The low-overhead C++ monitor records four stop-sign and red-light latencies:

```text
camera frame -> YOLO publish -> stable target -> behavior override -> stopped
```

Start it from the repository root after building and sourcing the workspace:

```bash
ros2 run carkit_latency_monitor stop_latency_monitor
```

It subscribes only to `/perception/latency_trace`,
`/behavior/stop_latency_trace`, and `/odom`. The default CSV is written under
`log/latency/`. Override the path when needed:

```bash
ros2 run carkit_latency_monitor stop_latency_monitor --ros-args \
  -p output_path:=log/latency/cpu_frequency_run.csv
```

Start the monitor before entering `AUTO_DRIVE`. Each transition from a
non-autonomous state into `AUTO_DRIVE` automatically resets the stop tracking,
starts a new latency trial, and increments `trial_id`. Repeated `AUTO_DRIVE`
state messages do not reset an active trial.

The manual reset service remains available while outside `AUTO_DRIVE`, but is
not required for the normal switch-into-auto experiment flow:

```bash
ros2 service call /behavior/reset_stop_latency_trial std_srvs/srv/Trigger "{}"
```
