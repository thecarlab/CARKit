<div align="center">
  <img src="docs/logo.jpeg" alt="CARKit logo" width="64">
  <h1>CARKit</h1>
  <a href="https://www.thecarlab.org/">The CAR Lab</a>
</div>

CARKit is a ROS 2 Humble stack for small Ackermann autonomous vehicles. The
current workflow uses one Docker image, a mounted workspace, VESC odometry,
Nav2 mapping/navigation, camera perception, behavior overrides, and one final
control arbiter.

## 🧩 Supported Platform

- 🚀 [NVIDIA Jetson Orin Nano](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/) with JetPack 6.x / L4T 36.x
- 🐳 Docker with [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- 📡 [SLAMTEC SLLiDAR/RPLIDAR](https://www.slamtec.com/en/support) publishing `/scan`
- 📷 [Intel RealSense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html) color camera
- 🏎️ Ackermann/F1TENTH-style vehicle with [VESC](https://vesc-project.com/) odometry

ROS 2 and CARKit dependencies run inside `ariiees/carkit:latest`. The host
only needs JetPack, Docker, Git, display access for RViz, and device access.

## ⚙️ Setup

On the Jetson host:

```bash
git clone https://github.com/thecarlab/CARKit.git
cd CARKit
docker pull ariiees/carkit:latest
./docker/run_jetson.sh
```

Inside the container:

```bash
./docker/build_workspace.sh
source install/setup.bash
```

`build_workspace.sh` fetches vendored sensor repos and builds the mounted
workspace at `/workspaces/CARKit`.

USB reminder before launching sensors:

- Connect the RealSense camera to a high-speed USB bus. If perception is
  unstable or images stop publishing, move the camera to a port that shows
  `10000M` or `5000M` in `lsusb -t`.
- Keep the lidar and VESC on separate stable USB connections when possible.
- Inside Docker, confirm devices are visible before launch:

```bash
lsusb -t
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

## 🕹️ Manual Driving And Mapping Control

For manual driving, mapping, and vehicle checks, launch human control directly:

```bash
ros2 launch carkit_human_control joystick.launch.py
```

This launches joystick teleop, VESC, odometry, and the legacy mux path from
`/teleop` to `/ackermann_cmd`.


Start human control as shown above, then launch mapping:

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=mapping
```

Drive through the environment, then save the occupancy map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspaces/CARKit/map/map
```

Maps belong in the repository's top-level `map/` folder.

## 🤖 Autonomous Driving

Start human control with the legacy mux output remapped away from
`/ackermann_cmd`, start the control center, then launch Nav2:

```bash
ros2 launch carkit_human_control joystick.launch.py \
  vehicle_command_topic:=/ackermann_mux_unused
```

```bash
ros2 launch carkit_control_center control_center.launch.py
```

```bash
ros2 launch carkit_navigation navigation.launch.py \
  mode:=navigation \
  start_command_mux:=false \
  map:=/workspaces/CARKit/map/map.yaml
```

In RViz, set **2D Pose Estimate**, wait for AMCL to converge, then send a
**Nav2 Goal**. Press the joystick mode toggle to enter `AUTO_DRIVE`; the
current default is `mode_toggle_button: 10` in
`f1tenth_stack/config/joy_teleop.yaml`.

The main map is selected above. To use the 3F example map instead, pass:

```bash
map:=/workspaces/CARKit/map/map_3f.yaml
```

### 👁️ Perception And Behavior

Start the color-only RealSense driver and typed 2D YOLO perception together:

```bash
ros2 launch carkit_perception perception.launch.py
```

Start behavior overrides:

```bash
ros2 launch carkit_behavior behavior_center.launch.py
```

Behavior logic only affects commands while the control center is in
`AUTO_DRIVE`.

## 🗂️ Repository Layout

```text
carkit/
  control/       human teleop, behavior layer, autonomous command arbiter
  navigation/    SLAM Toolbox, AMCL, Nav2, Twist-to-Ackermann bridge
  perception/    color-only YOLO and typed 2D detection messages
  sensors/       sensor driver fetch notes and transform nodes
  vehicle/       vendored F1TENTH/VESC vehicle stack
  tools/         classroom utilities and demos
map/             all occupancy maps
docker/          image, run, build, and publish scripts
docs/            troubleshooting and diagrams
```

## 📚 More Docs

- [Control](carkit/control/README.md)
- [Navigation](carkit/navigation/README.md)
- [Perception](carkit/perception/README.md)
- [Sensors](carkit/sensors/README.md)
- [Vehicle](carkit/vehicle/README.md)
- [Docker](docker/README.md)
- [Troubleshooting](docs/troubleshooting.md)
