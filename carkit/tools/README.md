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

## System Metrics Plotter

`system_metrics_plotter.py` turns CARKit system, node-CPU, topic-rate, and
pipeline-event CSV files into interactive plots and publication-ready
figures. It is a standalone Tkinter + Matplotlib program: it does not use ROS
and does not require `colcon build`.

Open one experiment in the GUI:

```bash
cd /workspaces/CARKit
python3 carkit/tools/system_metrics_plotter.py \
  log/system_monitor/low_battery_test.csv
```

Compare multiple experiments by elapsed time:

```bash
python3 carkit/tools/system_metrics_plotter.py \
  run_1200.csv run_1500.csv run_1728.csv
```

The left panel controls experiment names, metric selection, time range,
language, figure width, and PNG resolution. The range slider and Start/End
fields select the samples used for both plotting and statistics. The
Matplotlib toolbar and mouse wheel change only the current view, so zooming
does not silently change the statistical range. In timestamp mode, Start and
End accept ISO timestamps such as `2026-07-22T21:30:00-04:00`.

Each metric uses a separate subplot with units on its axis. CPU frequencies
are converted from Hz to MHz, and `cpu_mean_freq_mhz` is derived from all
online CPU cores at each sample. Blank or `offline` values are ignored rather
than interpolated. `VDD_CPU_GPU_CV` is labelled as the combined CPU/GPU/CV
power rail; it is not presented as CPU-only power.

When `pipeline_events.csv` is loaded with one or more metrics CSV files, its
events are overlaid on the existing subplots as color-coded vertical dashed
lines. Short event labels are staggered on the first subplot so nearby events
remain readable; the event file does not create a separate subplot or
statistics rows. The GUI selects every available event type by default. Use
**Pipeline event markers** to select only the event types that should be drawn,
then choose **Redraw**.

Use **Export Figure** (or the toolbar Save button) for PDF, SVG, or PNG.
PDF and SVG preserve vector graphics; PNG uses the selected 300 or 600 DPI.
Every GUI figure export also writes a JSON recipe beside the figure. Use
**Export Statistics** to write both a long-format CSV and a `booktabs` LaTeX
table containing count, mean, sample standard deviation, min, median, P95,
and max.

For a reproducible headless export:

```bash
python3 carkit/tools/system_metrics_plotter.py \
  run_1200.csv run_1728.csv pipeline_events.csv \
  --no-gui \
  --metrics vdd_in_voltage_v vdd_in_power_w \
            vdd_cpu_gpu_cv_power_w cpu_mean_freq_mhz \
  --event-types auto_drive_entered vehicle_stopped \
  --time-mode elapsed \
  --time-unit minutes \
  --start 10 \
  --end 120 \
  --figure-width double \
  --output figures/power_comparison.pdf \
  --stats-output figures/power_comparison_stats.csv \
  --latex-output figures/power_comparison_stats.tex
```

Omit `--event-types` to draw every event type, or pass `--no-events` to load
the event CSV without drawing markers.

To reproduce a GUI export, the recipe restores the absolute input paths,
experiment names, metrics, time range, language, size, title, and DPI. With no
explicit output option, headless recipe mode writes a PDF with the same stem
as the JSON file:

```bash
python3 carkit/tools/system_metrics_plotter.py \
  --no-gui \
  --recipe figures/power_comparison.json
```

GUI mode needs a working desktop/X11 display. For SSH sessions without display
forwarding, use `--no-gui`; Matplotlib then uses its non-interactive Agg
canvas.

## Full CPU Load Test

The standalone load generator starts one process per CPU available to the
container and pins each process to a different CPU. It does not set the CPU
clock; Linux and the Jetson firmware continue to control DVFS and throttling.

Run the system monitor and load generator in two container terminals. Start
CSV recording in the first terminal:

```bash
docker exec -it carkit bash
cd /workspaces/CARKit
python3 carkit/tools/jetson_system_monitor.py mode2 \
  --output /workspaces/CARKit/log/system_monitor/cpu_full.csv
```

Start a five-minute full-CPU run in the second terminal:

```bash
docker exec -it carkit bash
cd /workspaces/CARKit
python3 carkit/tools/cpu_full_test.py --duration 300
```

The default software temperature limit is 90 C. Change it with `--max-temp`,
or deliberately disable the software limit with `--max-temp 0`. Set
`--duration 0` to run until `Ctrl+C`.

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
