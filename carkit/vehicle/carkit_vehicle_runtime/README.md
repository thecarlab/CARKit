# CARKit internal vehicle runtime

Native implementations of the measured-motion and actuator adapters. These are
infrastructure, not student exercise files. The Python course implementations
and their stable topics remain in their existing education packages.

- `imu_rate_limit`: newest fresh gyro and accelerometer samples at 50 Hz; keeps
  original values, frame IDs, covariance, and timestamps; never repeats samples.
- `realsense_wheel_odometry`: actual VESC ERPM translation and transformed
  RealSense gyro rotation, with stationary startup bias estimation. Pauses on
  missing feedback/TF and does not integrate across gaps. Publishes no TF itself.
- `vesc_speed_control`: bounded feedforward + PI before the vendor RPM converter.
  Zero commands bypass the timer; stale/invalid/faulted feedback produces zero.

The latter two targets are built when VESC messages are available and
`CARKIT_ENABLE_VESC_TELEMETRY=ON`. OSRacer builds set this option to `OFF`.
`docker/build_workspace.sh` installs the chosen chassis message package first.

The pure C++ motion tests run through CTest. The ROS contracts in
`carkit/core/carkit_bringup/test` run against the Python references by default.
Set `CARKIT_NATIVE_RUNTIME_DIR` to this package's installed executable directory
to exercise the same contracts against C++. These tests require ROS domain 199
and localhost-only discovery and must never be run in the vehicle's domain.
