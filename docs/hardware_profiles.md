# Hardware profiles and control safety

`carkit_bringup` selects `config/hardware/<chassis>.yaml` for `osracer`,
`f1tenth`, or `vesc_rplidar`. A custom assembled vehicle can use a copy:

```bash
ros2 launch carkit_bringup carkit.launch.py chassis:=osracer \
  hardware_config:=/workspaces/CARKit/my_vehicle.yaml
```

The profile's `chassis` must match the selected adapter. It defines wheelbase,
forward/reverse limits, steering limit, footprint size and centre in `base_link`,
device identifiers, sensor mounts, and optional joystick mappings. VESC profiles
also contain motor/servo calibration. The old `config/vesc_rplidar.yaml` has been
replaced by `config/hardware/vesc_rplidar.yaml`; update calibration there.

Mounts are `[x, y, z, yaw, pitch, roll]` in metres/radians relative to `base_link`.
The bundled profiles preserve existing calibration and geometry where available;
VESC/RPLidar mounts and VESC camera mounts are nominal and require measurement.
Shared computer hardware does not imply shared wheelbase or sensor placement.
OSRacer and legacy F1TENTH footprints preserve the existing Nav2 rectangle, whose
centre is ahead of the base origin. The scan filter uses that same rectangle and
transforms laser returns into the base frame at the scan timestamp.

Defaults are conservative: OSRacer and legacy F1TENTH allow 0.8 m/s, VESC/RPLidar
0.5 m/s. OSRacer steering is limited to 0.34 rad; both VESC selections use 0.27 rad
to stay inside the bundled servo limits. Physically measure wheelbase, steering
centre/gain, footprint and sensor poses before treating these as calibrated.
Motor/servo direction and firmware communication timeout still require a vehicle
bench test. Software tests do not establish braking distance or sensor calibration.

## Command ownership

The integrated launch defaults to `manual_input:=keyboard` and starts no joystick.
The WebUI publishes bounded `/teleop` commands while WASD is held. Caps Lock sends
an autonomy request; the displayed mode is confirmed by the arbiter. Space and the
Emergency stop button latch a stop and clear the autonomous selection. Both WebUI
and joystick reset return to manual without restoring a buffered command. Autonomy
requests received during the stop are ignored; enabling it requires a new request
after reset.
Keyboard input is disabled while typing, changing views, or configuring the stack.

After sign-in, Take control grants one browser operator ownership. Disconnect,
Release control, or three seconds without its heartbeat releases ownership.
A zero motion command stops the vehicle without giving up operator ownership. No server timer republishes old
browser commands. The arbiter zeros stale manual commands after 300 ms. A browser
losing focus clears held keys and attempts an immediate zero; a broken connection
is covered by the arbiter timeout. Autonomous operation does not depend on holding
a browser key or keeping the browser focused.

Select `manual_input:=joystick` to start the common joystick adapter instead. Its
axes, mode-toggle and E-stop buttons come from the hardware profile. Both input
methods feed the same arbiter. The arbiter is retained when autonomous control is
disabled; autonomy requests are then ignored. Vendor joystick/mux launch files
are not part of the integrated VESC path.

OSRacer consumes only `/ackermann_cmd` by default. Standalone Twist diagnostics
can explicitly set `enable_cmd_vel:=true` on its driver, but must not run alongside
the integrated arbiter. Invalid numeric commands stop both motion axes at the
driver boundary. An active behavior override remains latched if its command or
heartbeat is lost; an explicit inactive message releases it. Watchdog freshness
inside the arbiter uses a monotonic clock.

## Localization and feedback

For both `f1tenth` and `vesc_rplidar`, `/odom` is published by
`realsense_wheel_odometry`. Translation uses **measured VESC ERPM** from
`/sensors/core`, converted with the hardware profile's speed gain. Rotation uses
the RealSense `/camera/camera/gyro/sample` angular velocity, rotated from its
optical sensor frame into `base_link` through TF. The node has no drive, servo,
or steering-command subscription. The vendor's command-steering odometry node
is no longer launched. `odom_tf_broadcaster` remains the sole `odom -> base_link`
TF publisher; AMCL uses LiDAR and this measured local motion for `map -> odom`.

This is measured planar dead reckoning, not IMU-only position estimation or
visual-inertial odometry. Accelerometer integration is not used as a substitute
for sustained speed. Wheel slip, RPM calibration error, gyro bias, and incorrect
sensor mounts can still cause drift; AMCL provides map correction. At startup,
hold the vehicle still for two seconds: near-zero measured RPM and stable gyro
and accelerometer readings establish the session's gyro bias. No bias adaptation
runs while driving. An explicitly measured body-frame `gyro_bias_z` can instead
be supplied with `calibrate_gyro_bias:=false`; steering calibration is not an IMU
bias measurement. The vehicle footprint and sensor offsets still require measurement.

Both sources require advancing timestamps and fresh reception (default 0.25 s).
Missing IMU mounting TF, stale gyro/RPM, invalid values, or a VESC fault pauses
odometry. Recovery does not integrate across the unseen interval and increases
pose uncertainty. The VESC autonomous arbiter also requires fresh `/odom`
(default 0.5 s); manual driving remains available without localization. There is
no fallback to commanded steering. Turning off **camera video** leaves RealSense
gyro/accelerometer enabled; turning off the sensors component removes the IMU
and therefore prevents VESC autonomous driving.

OSRacer retains its own odometry adapter and does not start RealSense odometry.
Its separate IMU integration can be configured when that sensor is selected.
The WebUI map uses `/odom` plus AMCL correction. Its motion metrics display
measured speed and measured yaw rate, not requested speed or requested steering.
Yaw rate is body rotation in degrees/second, not a measurement of wheel steering
angle. Missing motion feedback blanks the metrics and marks the map stale.

The ADA planner transforms the odometry position into the goal frame before
creating a path. Its controller transforms the path into the current odometry
frame at the odometry timestamp, and stops when odometry reception is older than
`odom_timeout_sec` (default 0.5 s) or the transform is unavailable. Invalid goals
clear the previous path. A map-frame goal requires a real `map -> odom` transform;
the guided planner does not substitute an identity transform or provide AMCL.

## Hardware-free regression checks

Use a sourced ROS 2 Humble environment and an unused localhost-only ROS domain.
Run the Python tests in each package separately (the legacy test directories
share the name `test`). The added checks cover malformed motor commands, stale
odometry, nonidentity and missing transforms, all hardware/input launch variants,
and rotated/offset LiDAR mounts. The control-center C++ tests exercise live ROS
messages for behavior loss, E-stop precedence and command timeouts.

```bash
node --test carkit/interface/carkit_webui/test/test_keyboard.cjs
```

After building the bridge and arbiter, run their hardware-free WebSocket pipeline:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187 \
CARKIT_BRIDGE_EXE="$PWD/install/carkit_web_bridge/lib/carkit_web_bridge/web_bridge_node" \
CARKIT_ARBITER_EXE="$PWD/install/carkit_control_center/lib/carkit_control_center/control_center_node" \
python3 -m pytest -q carkit/interface/carkit_web_bridge/test/test_keyboard_pipeline.py
```

That integration test starts no hardware drivers. It checks bounded commands,
idle-command expiry, competing browser sessions, disconnect stop, autonomy,
emergency stop, and reset to manual. It requires `pytest` and `websocket-client`.


## Perception source freshness

Each completed inference is published once. Output older than `output_timeout_sec`
(default 0.5 s) is discarded, and camera frames with missing, duplicate, future,
or expired timestamps are not processed. Skipped secondary-inference frames do
not reuse old sign detections as new observations. The behavior node rejects
repeated/non-increasing source stamps and holds a `PERCEPTION_STALE` zero override
when fresh perception is absent in autonomous mode. A fresh frame permits rule
evaluation again. This applies when the behavior module is enabled; disabling it
also disables its perception watchdog.

See [WebUI access](webui_access.md) for authentication, operator ownership, the
global emergency shortcut, and remote connection instructions.

## Chassis families and hardware configurations

The dashboard exposes two chassis families: OSRacer and F1TENTH / VESC.
RPLidar + RealSense is a hardware configuration of F1TENTH / VESC, not a
third chassis. The existing `vesc_rplidar` profile ID is retained for saved
installations and launch compatibility. The older `f1tenth` profile is retained only for legacy launch compatibility
and is not offered in the dashboard. The supported dashboard configuration
for this F1TENTH platform is RPLidar + RealSense. Profiles retain their distinct device paths,
geometry, and steering calibration; selecting a chassis family does not merge
or overwrite those values.

### VESC wheel-speed feedback

For F1TENTH and VESC/RPLIDAR, the command arbiter's `/ackermann_cmd`
feeds `vesc_speed_control`. At 50 Hz this adapter compares the target with
actual VESC ERPM converted to wheel speed using the selected hardware profile.
A feedforward plus PI correction produces `/ackermann_cmd/feedback`; only that
topic feeds the vendor Ackermann-to-ERPM converter. OSRacer is unchanged.

The initial gains are Kp=0.35 and Ki=0.6/s. Correction is limited to 0.5 m/s
in either direction before ERPM conversion, and the existing forward/reverse
RPM limits still apply. The adapter increases RPM for underspeed and reduces it
for overspeed, without reversing the commanded direction. Stop commands bypass
the timer and clear the integral. Direction changes, reductions in target
magnitude, invalid feedback, and timeouts also clear accumulated correction.
Conditional integration prevents saturation windup.

Motor commands become zero when either upstream commands or wheel feedback are
older than 250 ms. Feedback must have advancing recent ROS timestamps, finite
plausible speed, and no VESC fault. This does not replace the arbiter's ownership,
E-stop, acceleration limits, or the driver's hardware RPM bounds. It regulates
wheel speed rather than ground speed: tire slip or an incorrect ERPM-to-speed
calibration still requires independent measurement. Gains have isolated test
coverage; physical tracking performance must be checked on the vehicle.

RealSense gyro and accelerometer topics consumed by CARKit publish at 50 Hz.
The D435i on ADA2 advertises gyro rates of 200/400 Hz and accelerometer rates of
100/200/400 Hz; 50 Hz is not a native sensor mode. The camera publishes native
samples on `gyro/sample_raw` and `accel/sample_raw`. The `imu_rate_limit` adapter
forwards the newest fresh sample every 20 ms onto the existing `*/sample` topics,
retaining timestamps, frame IDs, values, and covariance. It never repeats an old
sample when the sensor stops. This reduces downstream processing rather than
the physical sensor sampling rate; it is decimation, not an anti-aliasing filter.

### Native internal runtime and image demand

The integrated F1TENTH/VESC launch uses the C++ executables in
`carkit_vehicle_runtime` for `imu_rate_limit`, `realsense_wheel_odometry`, and
`vesc_speed_control`. Their public ROS topics, 50 Hz cadence, mounting transforms,
stationary gyro-bias initialization, feedback deadlines, stop handling, and RPM
limits remain the same. Python implementations in bringup remain executable
references for differential tests; student packages in Intro2AV and ADA Academy
are not replaced or edited by this migration. OSRacer retains its own sensor and
chassis adapters. The chassis build installs VESC messages before configuring
these optional native VESC targets.

Reference perception in the integrated launch uses raw local camera frames,
avoiding JPEG decoding without changing the model, inference size, or rate.
The standalone perception launch still offers compressed transport for remote
consumers. The WebUI requests camera JPEGs only while the Overview camera panel
is visible. The native bridge creates that ROS subscription on demand and drops
it after the final viewer leaves; retained map/hardware data remain available to
new viewers. Per-client throttling is checked before JSON/CBOR serialization.
