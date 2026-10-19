<div align="center">
  <img src="docs/logo.jpeg" alt="CARKit logo" width="80">
  
  <a href="https://www.thecarlab.org/">The CAR Lab</a>
</div>

**CARKit** is a modular ROS 2 middleware platform for autonomous driving education, developed by the Connected and Autonomous Research (CAR) Lab at the University of Delaware for ADA Academy and Intro2AV.

Designed around real autonomous vehicle workflows, CARKit provides a unified software stack for perception, navigation, planning, and control on small-scale Ackermann vehicles. The platform combines industry-standard ROS 2 tools with hands-on deployment on physical vehicles and simulation environments, enabling students to learn autonomous systems through practical experimentation.

## 🧩 Supported Platforms

- 🚀 NVIDIA Jetson Orin Nano (`aarch64`). The current vehicle deployment is
  validated on Jetson Linux R39.2.1.
- 📦 The published container uses NVIDIA L4T JetPack `r36.4.0` (JetPack
  6.1 userspace), Ubuntu 22.04, ROS 2 Humble, CUDA, and TensorRT.
- 🐳 Docker Engine with the
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  on the Jetson host.
- 🏎️ OSRacer chassis, LakiBeam lidar, UVC camera, and a CUDY router
  providing the vehicle's private `192.*` network.
- 🏎️ F1TENTH/VESC chassis, Hokuyo URG lidar, Intel RealSense camera,
  and a Jetson connected directly to eduroam with a `128.*` address.
- 🛰️ The generic navigation package also retains serial SLLidar/RPLIDAR
  support for other course vehicles.
- 🌐 A current Chromium, Chrome, Firefox, or Safari browser. The native bridge
  accepts up to five simultaneous WebUI clients by default.

ROS 2 and CARKit dependencies run inside `ariiees/carkit:latest`. The host
only needs compatible Jetson Linux, Docker, Git, and hardware access for the
selected chassis. ROS 2 Desktop, RViz, and rqt remain available inside the
image for debugging, but no ROS GUI is started automatically. The normal
interface is the lightweight CARKit WebUI on port `8080`. It draws the map,
path, lidar, camera/perception results, and Jetson/chassis telemetry through
the native C++ CARKit WebSocket bridge.

## ⚙️ Setup

### Install on the Jetson host

Run these commands in a terminal on the **Jetson host**, not inside Docker.
On a fresh Jetson, clone the deployment branch:

```bash
git clone --branch ada2026 https://github.com/thecarlab/CARKit.git ~/CARKit
cd ~/CARKit
```

For an existing checkout, update it first (preserve any local changes):

```bash
cd ~/CARKit
git pull --ff-only
```

Choose **one** setup command below. Replace the example ADA number with the
actual vehicle ID; the ID does not determine the chassis type.

| Hardware | Setup selection |
| --- | --- |
| OSRacer, LakiBeam LiDAR, UVC camera | `osracer` |
| VESC, RPLidar S3M1-R2, RealSense D435i | `vesc_rplidar` |
| Legacy F1TENTH/VESC, Hokuyo URG, RealSense | `f1tenth` |

#### OSRacer prerequisites

Before running setup, use NetworkManager on the Jetson to save the credentials
and establish the intended connections:

- **Onboard Wi-Fi:** the vehicle's `OSR-*` network, such as `OSR-F912`.
  Reserve this connection for the vehicle's local network; do not use it for
  remote SSH or dashboard access.
- **USB Wi-Fi:** `eduroam`, using the institution's eduroam configuration.
  Use this adapter's `128.*` address for SSH and the WebUI.

Check the assignments locally:

```bash
nmcli -t -f DEVICE,STATE,CONNECTION device
ip -4 -br address
```

Setup requires exactly one onboard and one USB Wi-Fi adapter, saved profiles
for eduroam and a unique `OSR-*` network, and any active Wi-Fi connection to
already match its intended network. It does not provision eduroam credentials
or disconnect an incorrectly assigned active connection automatically.

Install OSRacer, for example on ADA5:

```bash
./docker/setup_jetson.sh ADA5 osracer
```

The host setup binds the saved Wi-Fi profiles to their respective adapters,
disables autoconnect on other saved Wi-Fi profiles, and gives USB eduroam
IPv4 routes metric **50**. The reporter uses only the USB adapter's address;
it never falls back to the OSR or Ethernet address. These settings persist
across reboot. Existing local OSR and LiDAR routes remain in place.

#### VESC / RPLidar / RealSense

For ADA2's VESC + RPLidar S3M1-R2 + RealSense D435i configuration:

```bash
./docker/setup_jetson.sh ADA2 vesc_rplidar
```

See the [ADA2 setup and calibration instructions](docs/vesc_rplidar_setup.md).

#### Legacy F1TENTH

Use the actual vehicle ID in place of the example `ADA3`:

```bash
./docker/setup_jetson.sh ADA3 f1tenth
```

#### Reporter token and host services

The first run asks for the CARLab monitor token with hidden input. Obtain the
vehicle's token from the CARLab administrator. For a vehicle-specific token,
the fleet monitor Site must have the matching secret, for example
`CARKIT_REPORTER_TOKEN_ADA6` for vehicle ID `ADA6`. On the Jetson, setup stores
the token as `CARKIT_REPORTER_TOKEN` in the root-readable
`/etc/carkit-webmonitor.env` and reuses it on subsequent runs. Never put the
token in this repository or directly in a shell command.

`setup_jetson.sh` performs the host setup:

- installs missing Docker, Python, and HTTPS client packages;
- enables host time synchronization and configures the NVIDIA container runtime;
- pulls `ariiees/carkit:latest`;
- installs OSRacer USB, camera, LakiBeam network, and Wi-Fi profile settings
  when applicable;
- repairs the Richbeam network binding before every container start, even when
  its MAC-derived Linux interface name changes;
- installs and enables the CARKit container/WebUI and authenticated reporter
  services at boot; and
- starts the services and waits for the WebUI on port `8080`.

Run setup while the vehicle is stopped: it may restart Docker and the CARKit
service. After updating an existing checkout, you can reuse its saved vehicle
ID, chassis selection, and reporter token:

```bash
./docker/setup_jetson.sh
```

#### Build the selected chassis in the WebUI

Host setup starts the dashboard; **the chassis workspace still needs to be
installed/built**. Open the vehicle from the
[CARLab ADA Fleet Monitor](https://carlab-ada-monitor.udcarlab.chatgpt.site),
select the matching chassis, and click **Install / build selected chassis**.
For VESC/RPLidar hardware, select **VESC + RPLidar + RealSense**.

Use the vehicle's eduroam `128.*` address with plain HTTP and port `8080`,
for example `http://<vehicle-eduroam-ip>:8080/`. The viewing computer must have
a permitted network path to that address. Do not connect to OSR-F912 or use
an OSR/LiDAR `192.*` address for remote management. Campus wireless-client
restrictions can affect direct eduroam access even while internet access works.

The reporter sends an update once the container, dashboard, and synchronized
host clock are ready, then **every minute**. The monitor displays a vehicle as
online for **two minutes** after its latest report; this confirms recent
reporting, not inbound dashboard reachability. Failed reports retry every five
minutes for at most 20 consecutive attempts, then stop until the reporter
service or vehicle restarts.

For either chassis, the dashboard lets you choose the course profile,
independently override each algorithm, select startup components, launch,
stop, and inspect live results. The **Code editor** provides a read-only view
of the component-scoped reference implementation plus editable ADA and
Intro2AV workspaces, a file explorer, syntax diagnostics, and collaborative
editing for up to five users. The **Compile** page can
rebuild the entire repository or only perception, localization, control, or
planning. The next launch automatically sources the updated install overlay.

### Service management

The setup command enables both services at boot. They do not require a GNOME
login. Useful commands:

```bash
systemctl status carkit carkit-webmonitor
sudo systemctl restart carkit
sudo systemctl restart carkit-webmonitor
sudo systemctl stop carkit
journalctl -u carkit -f
journalctl -u carkit-webmonitor -f
```

To use a shell while the boot service or foreground WebUI is running:

```bash
docker exec -it carkit bash
```

To run only a shell with no WebUI, stop the service first:

```bash
sudo systemctl stop carkit
./docker/run_jetson.sh bash
```

Then, inside that container shell, use the command for the selected chassis.
For OSRacer:

```bash
./docker/install_carkit.sh osracer
```

For F1TENTH:

```bash
./docker/install_carkit.sh f1tenth
```

`install_carkit.sh` records the selected chassis in `.carkit/config.json`,
fetches the pinned optional F1TENTH source when needed, and builds the mounted
workspace at `/workspaces/CARKit`.

## 🎓 Course Profiles

- `reference`: native C++ planning, behavior, localization adapters, and
  control; Python is retained only for YOLO/TensorRT perception.
- `ada_high_school`: working guided planning, control, and perception from the
  student-owned `carkit_ada_academy` package.
- `intro2av`: safe ROS 2 boilerplates for planning, control, and perception in
  both Python and C++; topic names, timers, and the safety boundary remain.

Behavior planning remains enabled for every profile and lives in the planning
layer. Course selection changes planning, control, and perception ownership
together; each ownership selector can still override one component without
modifying the reference source.

Switching a component in the WebUI never copies over the reference source.
Student work lives in the separate `carkit/education/carkit_ada_academy`,
`carkit/education/carkit_intro2av`, and `carkit_intro2av_cpp` packages;
reference implementations stay in their production packages. Changing the
course selector chooses Intro2AV Python initially, while each Algorithm
Ownership selector can independently switch to Intro2AV C++. See
[`docs/course_profiles.md`](docs/course_profiles.md).

### Using the WebUI

Use the vehicle's **eduroam `128.*` address on port `8080`** from a computer
with a permitted campus-network path. For OSRacer, remote access uses USB
Wi-Fi; onboard OSR Wi-Fi remains reserved for the vehicle network. The
Overview provides the map, LiDAR, perception overlay, launch controls,
telemetry, and goal/pose tools without an RViz or Foxglove process.

To localize the vehicle, click **Set initial pose**, press on its current map
position, drag toward the vehicle's forward direction, and release. Goal poses
use the same position-and-heading gesture. Drag the map to rotate, Shift-drag
to pan, and use the mouse wheel to zoom.

USB reminder before launching sensors:

- Connect the OSRacer USB hub and verify the UVC camera alias. The LakiBeam is
  exposed over USB Ethernet at `192.168.8.2`; the setup script assigns its
  Richbeam adapter `192.168.8.1/24` with no default route.
- On the Jetson host, confirm the devices and direct LiDAR route before launch:

```bash
lsusb -t
ls -l /dev/osrbot_base /dev/osrbot_usb_cam
ip -4 -brief address
ip route get 192.168.8.2
ping -c 1 192.168.8.2
```

The direct `ros2` examples below are advanced alternatives to the WebUI. Run
them from a shell inside the active container (`docker exec -it carkit bash`)
and stop any WebUI launch session that already owns the same hardware.

## 🕹️ Keyboard Driving And Autonomous Control

Sign in with the private `.carkit/access.key` file on the vehicle computer, then
select **Take control**. See [WebUI access and remote connections](docs/webui_access.md)
for SSH forwarding and explicit network binding.

Both chassis use **keyboard control by default** in the focused WebUI Overview:

- Hold **W / S** to move forward / reverse (up to 0.4 m/s).
- Hold **A / D** to steer left / right; combine with W or S to turn.
- Release the keys to stop. Leaving the window, opening the editor/configuration,
  or losing the connection stops manual motion; press the keys again to resume.
- Press **Caps Lock** once to enable or disable autonomous driving. The displayed
  mode comes from the control center; holding Caps Lock does not repeatedly toggle it.
- Press **Space** on Overview or **Ctrl + Shift + Space** from any page, or click
  the persistent **Emergency stop** button, to latch a stop. **Reset to manual**
  clears it without starting motion. Caps Lock cannot clear an emergency stop.

Typing in the code editor, terminal, or form fields does not drive the vehicle.
Only one browser can supply keyboard motion at a time. The arbiter remains active
in manual-only sessions, even when the **autonomous control** module is unchecked.
Joystick input is optional: select **Manual input → Joystick** before launching,
optionally customize the button mappings in the vehicle hardware profile, or pass
`manual_input:=joystick` to the integrated launch.

For a full reference navigation session inside the container:

```bash
ros2 launch carkit_bringup carkit.launch.py profile:=reference chassis:=osracer
# Or select chassis:=f1tenth / chassis:=vesc_rplidar for the VESC vehicles.
```

Open the WebUI, set the vehicle's initial pose, send a goal, and use Caps Lock to
switch to autonomous driving. Switching back to manual requires a new WASD press.
The selected course profile determines which planning and control algorithms run.
ADA algorithms now require fresh odometry and a valid transform between the goal,
path, and odometry frames; a map-frame goal needs a localization provider publishing
`map -> odom`. Missing feedback or transforms produce a stop, not an assumed pose.

For keyboard mapping, select the reference profile and use
`navigation_mode:=mapping`. Save the map from a container terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f /workspaces/CARKit/map/test
```

Per-vehicle geometry, calibrated gains, device identifiers, mounting transforms,
and limits are in [the hardware profiles](carkit/core/carkit_bringup/config/hardware).
See [hardware profiles and safety validation](docs/hardware_profiles.md).
The conservative default forward limits are 0.8 m/s for OSRacer and the legacy
F1TENTH selection, and 0.5 m/s for VESC + RPLidar. Keyboard speed is additionally
limited to 0.4 m/s. Changes to a hardware profile are applied together to the
hardware driver, navigation, student control parameters, arbiter, and WebUI bridge.

### 👁️ Perception And Behavior

With the selected sensor bringup publishing the camera, start typed 2D YOLO
perception:

```bash
ros2 launch carkit_perception perception.launch.py
```

The WebUI shows the annotated perception stream directly. Its
configuration drawer can select generic COCO, traffic-sign-only, combined, or
a custom Jetson TensorRT model. Camera publication and perception inference
both target 10 Hz while discarding stale frames to keep the live view current.
In the map/lidar panel, drag to rotate,
Shift-drag to pan, use the mouse wheel to zoom, and double-click to reset.

Start behavior overrides separately:

```bash
ros2 launch carkit_behavior behavior_center.launch.py
```

Behavior logic only affects commands while the control center is in
`AUTO_DRIVE`.

## 🗂️ Repository Layout

```text
carkit/
  core/          one carkit_bringup entry point, profiles, stable interfaces
  education/     separate ADA Academy and Intro2AV algorithm packages
  interface/     WebUI, collaborative editor, and native C++ ROS bridge
  control/       human teleop and autonomous command safety arbiter
  planning/      scalable road-behavior planning rules
  navigation/    SLAM Toolbox, AMCL, Nav2, Twist-to-Ackermann bridge
  perception/    Python YOLO/TensorRT inference and typed 2D detections
  sensors/       camera, scan filter, sensor transforms, and vendor adapters
  vehicle/       OSRacer integration and pinned F1TENTH vendor selection
  tools/         classroom utilities and demos
map/             all occupancy maps
docker/          image, run, build, and publish scripts
docs/            troubleshooting and diagrams
```

All supported workflows start through one ROS entry point inside the one
`carkit` Docker container:

```bash
ros2 launch carkit_bringup carkit.launch.py \
  profile:=intro2av chassis:=f1tenth
```

Vendor package names such as `osracer_base`, `f1tenth_stack`, and `vesc_driver`
remain unchanged so upstream updates and licenses stay traceable. They are
isolated behind the CARKit launch and topic contract; optional F1TENTH sources
are fetched at their pinned revisions only when that chassis is installed.

## 📚 More Docs

- [Control](carkit/control/README.md)
- [Navigation](carkit/navigation/README.md)
- [Perception](carkit/perception/README.md)
- [Sensors](carkit/sensors/README.md)
- [Vehicle](carkit/vehicle/README.md)
- [Docker](docker/README.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Architecture and course profiles](docs/course_profiles.md)
- [WebUI design](docs/webui_design.md)
