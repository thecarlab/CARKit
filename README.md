<div align="center">
  <img src="docs/logo.jpeg" alt="CARKit logo" width="80">
  
  <a href="https://www.thecarlab.org/">The CAR Lab</a>
</div>

**CARKit** is a ROS 2 platform for autonomous-driving education at the
University of Delaware's CAR Lab. It combines vehicle hardware, perception,
localization, planning, and control with editable ADA Academy and Intro2AV
course implementations.

The normal interface is a browser dashboard on port **8080**. ROS 2 and vehicle
dependencies run in the `carkit` Docker container; RViz and rqt are available
for debugging but are not started automatically.

## Supported hardware

| Chassis | Sensors | Setup / launch selector |
| --- | --- | --- |
| OSRacer | LakiBeam LiDAR, UVC camera | `osracer` |
| F1TENTH | VESC, RPLidar S3M1-R2, RealSense D435i | `vesc_rplidar` |

`vesc_rplidar` is the code selector for the F1TENTH hardware configuration,
not a third chassis type.

The vehicle computer is an NVIDIA Jetson Orin Nano (`aarch64`). Vehicle
deployments have been validated on Jetson Linux R39.2.1. The base image,
`ariiees/carkit:latest`, uses L4T `r36.4.0` / JetPack 6.1 userspace, Ubuntu 22.04,
ROS 2 Humble, CUDA, and TensorRT. Host and container versions are distinct.
Use a current browser; the native WebSocket bridge supports up to five clients.

## Installation

### 1. Prepare the host network and token

Run host setup on the **Jetson**, not inside Docker. Have Git, sudo access,
internet access, and the vehicle's CARLab monitor token available. Setup
installs missing host dependencies and configures the NVIDIA Docker runtime.

For **OSRacer**, save the network credentials in NetworkManager and establish:

| Adapter | Network | Purpose |
| --- | --- | --- |
| Onboard Wi-Fi | The vehicle's `OSR-*` SSID, such as `OSR-F912` | Local vehicle network |
| USB Wi-Fi | `eduroam` | SSH, dashboard access, and internet |

Verify the assignments locally:

```bash
nmcli -t -f DEVICE,STATE,CONNECTION device
ip -4 -br address
```

OSRacer setup requires one onboard and one USB Wi-Fi adapter, a unique saved
`OSR-*` network, and a saved eduroam profile. Any active connection must already
match its intended network. Setup does not provision eduroam credentials or
switch an incorrectly connected adapter automatically.

Setup binds the profiles to their adapters, disables other Wi-Fi profiles'
autoconnect, and gives USB eduroam IPv4 routes metric **50**. It preserves
local OSR and LiDAR routes and pins IP reporting to USB Wi-Fi. These settings
persist across reboot. Do not use the OSR connection for remote management.
See [OSRacer Wi-Fi assignment](docker/README.md#osracer-wi-fi-assignment).

For **F1TENTH**, connect the Jetson to eduroam and follow the
[F1TENTH hardware and calibration guide](docs/vesc_rplidar_setup.md).

Obtain the reporter token from the CARLab administrator. A vehicle-specific
token must also be configured on the monitor Site under
`CARKIT_REPORTER_TOKEN_ADA<number>`. Setup prompts for it with hidden input and
stores it as `CARKIT_REPORTER_TOKEN` in `/etc/carkit-webmonitor.env`, readable
only by root. Do not put tokens in source files or shell commands.

### 2. Clone and run host setup

On a fresh Jetson:

```bash
git clone --branch ada2026 https://github.com/thecarlab/CARKit.git ~/CARKit
cd ~/CARKit
```

Run **one** command for the actual chassis, replacing the example vehicle ID:

```bash
# OSRacer example
./docker/setup_jetson.sh ADA5 osracer

# F1TENTH example — run this instead for VESC/RPLidar/RealSense hardware
./docker/setup_jetson.sh ADA2 vesc_rplidar
```

Run setup while the vehicle is stopped: it may restart Docker and CARKit.
It enables time synchronization, prepares the container and hardware rules,
and installs the dashboard and reporter as boot services. No desktop login
is required. OSRacer setup also configures its USB devices, camera alias, and
LakiBeam Ethernet interface.

### 3. Open the dashboard and build the chassis

Host setup starts the WebUI; **building the selected chassis is a separate step**.
On the Jetson, open `http://127.0.0.1:8080/`. By default, access requires the
key in `~/CARKit/.carkit/access.key`.

For remote access with the default localhost binding, forward both ports from
your viewing computer. Replace `USER` and `VEHICLE_EDUROAM_IP`:

```bash
ssh -L 8080:127.0.0.1:8080 -L 9090:127.0.0.1:9090 USER@VEHICLE_EDUROAM_IP
```

Then open `http://127.0.0.1:8080/` on that computer. Fleet deployments explicitly
configured for network access can instead use the link in the
[CARLab ADA Fleet Monitor](https://carlab-ada-monitor.udcarlab.chatgpt.site).
Direct links use `http://<vehicle-eduroam-ip>:8080/`; they require a permitted
network path and a network-accessible dashboard binding. See
[WebUI access configuration](docs/webui_access.md) for binding and sign-in options.

In **Configure**, select the chassis family and hardware configuration. For
F1TENTH, select **F1TENTH / VESC** and the VESC/RPLidar/RealSense configuration.
Click **Install / build selected chassis** and wait for completion before launch.
OSRacer displays **No calibration needed**; F1TENTH has guided steering
calibration described in its hardware guide.

### Updating an existing installation

Preserve local work before pulling, and stop vehicle operation before setup:

```bash
cd ~/CARKit
git pull --ff-only
./docker/setup_jetson.sh
```

Setup reuses the saved vehicle ID, chassis, and token, or prompts for missing
values. Rebuild affected packages through the WebUI after source updates.
The next launch uses the updated install overlay.

## Daily operation

In **Configure**, select a course profile, algorithm implementations, and the
components to launch. The Overview shows the map, LiDAR, camera/perception,
vehicle telemetry, and runtime health. Components, including behavior planning,
can be enabled or disabled as needed.

For localization, choose **Set initial pose**, press at the vehicle's map
position, drag toward its forward direction, and release. Goal poses use the
same position-and-heading gesture. Drag the map to rotate, Shift-drag to pan,
scroll to zoom, and double-click to reset the view.

### Driving controls

After signing in if required, choose **Take control**. Only one browser owns
motion control at a time. Keyboard driving works in the focused Overview:

| Input | Action |
| --- | --- |
| Hold W / S | Forward / reverse, up to 0.4 m/s |
| Hold A / D | Steer left / right; combine with W / S to turn |
| Release keys | Stop manual motion |
| Caps Lock | Request autonomous/manual mode toggle |
| Space on Overview, or Ctrl + Shift + Space anywhere | Latch emergency stop |
| Reset to manual | Clear emergency stop without starting motion |

Losing focus, opening the editor/configuration, or losing the connection stops
manual motion. Typing in editors or forms does not drive the vehicle. Autonomous
operation is managed by the control arbiter; Caps Lock cannot clear an emergency
stop. Joystick input is available through **Manual input → Joystick**.

Hardware profiles define geometry, transforms, device identifiers, and limits.
The default forward limits are 0.8 m/s for OSRacer and 0.5 m/s for F1TENTH;
keyboard driving has its own 0.4 m/s cap. See
[hardware profiles and safety validation](docs/hardware_profiles.md).

### Course profiles and editing

| Profile | Purpose |
| --- | --- |
| `reference` | Reference implementations, primarily native C++; YOLO/TensorRT perception uses Python |
| `ada_high_school` | Guided student algorithms in `carkit_ada_academy` |
| `intro2av` | Student implementations in Python and C++ |

The **Code editor** separates read-only reference code from editable student
workspaces and supports collaborative editing. Selecting a course or algorithm
does not overwrite source files. **Compile** rebuilds the repository or selected
components. See [course profiles](docs/course_profiles.md) for package ownership
and implementation selection.

## Services and fleet reporting

Run these commands on the Jetson host:

```bash
systemctl status carkit carkit-webmonitor
journalctl -u carkit -f
journalctl -u carkit-webmonitor -f
sudo systemctl restart carkit-webmonitor
# Stop vehicle operation before restarting or stopping the runtime:
sudo systemctl restart carkit
sudo systemctl stop carkit
```

The reporter waits for the container, local dashboard, and synchronized clock,
then reports **every minute**. The monitor marks a vehicle online for **two
minutes** after its latest report. That means recent outbound reporting, not
verified inbound SSH or dashboard access. Failures retry every five minutes,
up to 20 consecutive attempts, then stop until the service or vehicle restarts.
For OSRacer, no USB Wi-Fi address means no report; there is no interface fallback.

## Command-line alternatives

These commands are for debugging or terminal-based operation. Stop any WebUI
launch session using the same hardware before launching ROS nodes manually.
Enter the running container from the host:

```bash
docker exec -it carkit bash
```

Inside the container, install/build the selected chassis if needed:

```bash
cd /workspaces/CARKit
./docker/install_carkit.sh osracer
# For F1TENTH, use: ./docker/install_carkit.sh vesc_rplidar
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the reference stack:

```bash
ros2 launch carkit_bringup carkit.launch.py profile:=reference chassis:=osracer
# For F1TENTH, use chassis:=vesc_rplidar. For Intro2AV, use profile:=intro2av.
```

Direct bridge launches require the authentication environment described in
[WebUI access configuration](docs/webui_access.md). For mapping, add
`navigation_mode:=mapping` to the launch command. Save a map from another
container terminal with the ROS environment sourced:

```bash
ros2 run nav2_map_server map_saver_cli -f /workspaces/CARKit/map/test \
  --ros-args -p free_thresh_default:=0.196
```

The threshold preserves unknown gray pixels when the saved PGM map is reloaded.
See [navigation](carkit/navigation/README.md),
[perception](carkit/perception/README.md), and [control](carkit/control/README.md)
for subsystem-specific commands.

## Repository layout

```text
carkit/
  core/          integrated bringup, profiles, and interfaces
  education/     ADA Academy and Intro2AV student packages
  interface/     WebUI, collaborative editor, and native ROS bridge
  control/       manual input and command safety arbiter
  planning/      behavior planning
  navigation/    SLAM, AMCL, Nav2, and command adapters
  perception/    YOLO/TensorRT inference and typed detections
  sensors/       camera, scan filtering, and sensor adapters
  vehicle/       chassis integration
  tools/         classroom utilities
map/             occupancy maps
docker/          host setup, image, build, and service scripts
webmonitor/      fleet monitor and vehicle reporter
docs/            hardware guides, architecture, and troubleshooting
```

Vendor package names remain unchanged for upstream and license traceability.
Required external vehicle sources are fetched at pinned revisions during build.

## Further documentation

- [F1TENTH setup and calibration](docs/vesc_rplidar_setup.md)
- [WebUI access and operator control](docs/webui_access.md)
- [Hardware profiles](docs/hardware_profiles.md)
- [Course profiles and architecture](docs/course_profiles.md)
- [Sensors](carkit/sensors/README.md) and [vehicles](carkit/vehicle/README.md)
- [Docker and host setup](docker/README.md)
- [Fleet monitor](webmonitor/README.md)
- [Troubleshooting](docs/troubleshooting.md)
