# Vehicle adapters

CARKit supports one installed vehicle adapter at a time:

- `osracer`: serial chassis, LakiBeam lidar, and UVC camera.
- `f1tenth`: VESC chassis, URG lidar, and RealSense camera.
- `vesc_rplidar`: public ROS 2 VESC driver, RPLidar S3M1-R2, and RealSense D435i.
  See [the ADA2 setup guide](../../docs/vesc_rplidar_setup.md).

Select it in the WebUI or run `docker/install_carkit.sh osracer|f1tenth`.
Both adapters sit behind `carkit_bringup` and the same `/odom`, `/scan`,
`/camera/...`, `/teleop`, and `/ackermann_cmd` contracts. Student algorithms
must not import adapter packages.

The integrated OSRacer stack provides joystick input, the serial chassis
driver, odometry, IMU, battery state, LakiBeam lidar, UVC camera, and
CARKit-compatible command routing. F1TENTH sources are fetched only when that
adapter is installed.

All integrated driving uses the shared arbiter, including manual-only sessions.
Keyboard control in the WebUI is the default: hold WASD to drive, Caps Lock to
toggle autonomy, and Space to stop. Joystick input is optional with
`manual_input:=joystick`.

```bash
ros2 launch carkit_bringup carkit.launch.py chassis:=osracer
ros2 launch carkit_bringup carkit.launch.py chassis:=f1tenth
ros2 launch carkit_bringup carkit.launch.py chassis:=vesc_rplidar
```

Run only one selected adapter at a time. VESC drivers are launched explicitly;
the vendor joystick/multiplexer launch is not included. OSRacer defaults to
Ackermann-only input, so Nav2's `/cmd_vel` cannot bypass the arbiter.

Geometry, devices, sensor mounting transforms and calibrated VESC conversion
values are in `carkit/core/carkit_bringup/config/hardware/*.yaml`. These feed
navigation, student control geometry, scan filtering and command limits together.
The RPLidar profile's former `config/vesc_rplidar.yaml` was replaced by the hardware
profile. See [hardware profiles](../../docs/hardware_profiles.md) for customization,
calibration boundaries and regression checks.
