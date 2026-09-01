# CARKit learning architecture

CARKit has one container, one launch entry point, and one stable ROS 2 contract.
Hardware and course difficulty are runtime choices rather than separate forks.

```text
browser :8080
  -> CARKit WebUI
  -> carkit_bringup/carkit.launch.py
       -> selected platform adapter (OSRacer or F1TENTH)
       -> sensor startup
       -> reference C++ | ADA Python | Intro2AV Python/C++ planning
       -> reference C++ | ADA Python | Intro2AV Python/C++ control
       -> reference Python | ADA Python | Intro2AV Python/C++ perception
       -> carkit_control_center C++ safety arbiter
  <-> C++ WebSocket bridge :9090 (map, lidar, path, image, telemetry)
```

## Why the source is separated

Production nodes stay in `carkit/control`, `carkit/navigation`,
`carkit/planning`, and `carkit/perception`. Reference planning, behavior,
localization adapters, and control execute as C++; perception remains Python
because its TensorRT/YOLO integration depends on the Python model ecosystem.
Students do not edit those packages. Course-owned code is
in `carkit/education/carkit_ada_academy` (Python), `carkit_intro2av`
(Python), and `carkit_intro2av_cpp` (C++). All expose the same topics.
Selecting another implementation changes launch actions, not files, so a student can
return to the reference result immediately without losing work.

The command boundary is deliberately stricter than the other exercises.
Student controllers publish `/drive`; `carkit_control_center` is always the
final autonomous publisher to `/ackermann_cmd`. It retains stale-command
timeouts, limits, E-stop, and manual takeover.

## Profiles

| Profile | Planning | Control | Perception | Intended use |
| --- | --- | --- | --- | --- |
| `reference` | Nav2 + C++ behavior | C++ Nav2 bridge + arbiter | Python YOLO/TensorRT | Demonstration and comparison |
| `ada_high_school` | ADA guided path | ADA guided pure pursuit | Reference detector + ADA filter | Small algorithm/tuning changes |
| `intro2av` | Intro2AV Python boilerplate | Safe Intro2AV Python boilerplate | Typed Intro2AV Python boilerplate | Implement algorithms from scratch in either language |

Changing the course in the WebUI explicitly switches all three ownership
selectors. Each can then be overridden independently. There is no hidden
"course default" option; choose `intro2av_python` or `intro2av_cpp` per
component, and use `off` for isolated labs.

## Stable interfaces

The canonical list is
`carkit/core/carkit_bringup/config/interfaces.yaml`. The main student-facing
flow is:

```text
/camera/camera/color/image_raw -> perception -> /yolo/detections_2d
/odom + /goal_pose             -> planning   -> /plan
/odom + /plan                  -> control    -> /drive
/drive + /teleop               -> safety     -> /ackermann_cmd
```

Both hardware adapters must publish `/odom`, `/scan`, and camera data with the
canonical frames, and consume `/ackermann_cmd`. Platform-specific package and
device names are not used by student algorithms.

## CLI use

The WebUI is the normal course path. The same operation can be reproduced in a
container shell:

```bash
./docker/install_carkit.sh osracer
source install/setup.bash
ros2 launch carkit_bringup carkit.launch.py \
  profile:=ada_high_school \
  chassis:=osracer
```

Mix implementations without changing source:

```bash
ros2 launch carkit_bringup carkit.launch.py \
  profile:=intro2av \
  chassis:=f1tenth \
  planning:=reference \
  control:=intro2av_cpp \
  perception:=off
```

Use `start_chassis:=false`, `start_sensors:=false`, or the corresponding
planning/control/perception/behavior arguments for isolated and replay-based
labs.

## Chassis installation

`docker/install_carkit.sh` accepts exactly `osracer` or `f1tenth`. The OSRacer
source is maintained in this checkout. The F1TENTH/VESC source is fetched from
`thecarlab/ada_system` at the pinned commit recorded by the installer. Colcon
skips the unselected platform packages, and `.carkit/config.json` records the
active choice. The WebUI refuses to launch a different chassis until it has
been installed.

Underlying vendor package names are intentionally not renamed. Keeping names
such as `vesc_driver` and `osracer_base` preserves upstream compatibility;
`carkit_bringup` is the public CARKit-facing abstraction.

## Browser visualization

The dashboard uses a small static client and the dedicated C++ CARKit WebSocket
bridge. Rendering happens on the student's browser rather than in RViz on the
Jetson. It currently shows occupancy maps, global paths, lidar points, vehicle
pose, compressed camera frames, control mode, speed, steering, build progress,
and launch logs. Keep ports `8080` and `9090` on the classroom LAN only; the
dashboard does not provide internet-facing authentication.

Camera JPEGs and ROS messages use the bridge's binary CBOR transport instead of
base64 JSON.
Perception boxes are drawn over that camera stream in each browser from the
typed 10 Hz detection topic, avoiding a second annotated JPEG encode on the
Jetson. The taskbar reports the CARKit Docker cgroup CPU separately from the
host-wide CPU value. CPU is aggregate across online cores, where each core is
100% (600% maximum on the six-core Jetson); temperature and memory remain
host-wide chassis metrics.

## Browser compilation

The WebUI **Compile** page provides targets for the entire repository,
perception, localization, control, and planning. Component builds use the
implementation currently selected in Setup: for example, Intro2AV C++
perception builds `carkit_perception_msgs` and `carkit_intro2av_cpp`, while ADA
perception also builds its protected reference detector. Shared safety packages
are always included for control. The backend uses `colcon --packages-up-to`, so
workspace dependencies are rebuilt without compiling the unselected Python,
C++, ADA, or reference implementation.

Compilation stops the active session first so installed files are not replaced
under running nodes. The exact resolved package list and build output are
streamed into the page. Every later launch starts a fresh shell and sources
`install/setup.bash`, so no terminal re-source step is required.
