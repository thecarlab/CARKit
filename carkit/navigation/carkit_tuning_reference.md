# CARKit OSRacer and Navigation Tuning

## Command and Odometry Contract

CARKit publishes `ackermann_msgs/msg/AckermannDriveStamped` on
`/ackermann_cmd`. `osracer_base` converts speed in meters per second and
steering angle in radians to the legacy controller command
`v <speed_mps> <steering_degrees>`.

The legacy controller publishes separate odometry (`o`) and IMU (`i`) frames.
The driver maps these to `/odom` and `/imu/data`, with `base_link` as the
odometry child frame. CARKit navigation owns the `odom -> base_link` transform,
so OSRacer TF publication is disabled in the integrated bringup.

## Chassis Limits

The relevant launch file is
`carkit/vehicle/osracer/osracer_bringup/launch/bringup_launch.py`.

| Argument | Default | Guidance |
|---|---:|---|
| `wheelbase` | `0.325 m` | Measure axle-to-axle; affects Twist-to-Ackermann geometry |
| `forward_max_speed` | `0.8 m/s` | Raise only after measuring stop distance and odometry |
| `reverse_max_speed` | `0.8 m/s` | Keep conservative for indoor testing |
| `max_steering_angle` | `0.5236 rad` | Do not exceed the mechanical steering range |
| `cmd_timeout` | `0.5 s` | Keep short enough to stop promptly after publisher loss |

Modern Proto 1.1 firmware reports these capabilities automatically. Select it
with `protocol_mode:=modern`; do not copy legacy limits into a modern vehicle
profile.

## Nav2 Controller

Tune `carkit/navigation/carkit_amcl/config/nav2_params.yaml` only after manual
commands, stopping, `/odom`, `/scan`, and the TF tree are verified.

Start with Nav2 speed limits no higher than the OSRacer driver limit. Important
parameters include:

- `desired_linear_vel` for cruise speed.
- `min_lookahead_dist`, `max_lookahead_dist`, and `lookahead_time` for Regulated
  Pure Pursuit tracking.
- `velocity_smoother.max_velocity`, `max_accel`, and `max_decel` for command
  bounds and stopping behavior.
- Costmap `inflation_radius`, `cost_scaling_factor`, `obstacle_max_range`, and
  `raytrace_max_range` for clearance and obstacle response.

## Safe Validation Sequence

1. Run `./docker/setup_osracer_device.sh` on the host and restart Docker.
2. Launch `ros2 launch carkit_human_control joystick.launch.py`.
3. Confirm `/ackermann_cmd` is `AckermannDriveStamped` and `/odom` is updating.
4. Test straight motion at low speed, issue an explicit zero command, and
   measure actual travel against odometry.
5. Test low-speed left/right steering and confirm the sign convention.
6. Validate `odom -> base_link -> laser` before starting mapping or Nav2.
7. Increase autonomous limits in small steps while measuring stop distance.

Always keep the path clear and the emergency stop within reach during motion
tests.

## Footprint clearance and steering feasibility

Integrated bringup derives the Hybrid-A* minimum turning radius from the selected
hardware profile as `1.10 * wheelbase / tan(max_steering_angle)`. The 10% radius
reserve leaves room for tracking corrections; it does not change the actuator's
physical steering limit. For ADA2's 0.33 m wheelbase and 0.245 rad limit this is
1.452 m, versus the physical-model minimum of 1.320 m.

Both costmaps add 0.08 m of footprint padding. Their 0.80 m inflation radius and
2.5 cost scaling spread a soft wall-proximity cost across corridors. Inflation
is not an 0.80 m exclusion zone. The physical rectangle still comes from the
hardware profile, so measure its length, width and offset from `base_link`.
Padding cannot correct a misplaced or undersized physical footprint.

The Hybrid-A* smoother is disabled (`smooth_path: false` and
`smoother.max_iterations: 0` for Humble compatibility). Humble's interior smoothing
moves path points using point-cost checks without enforcing interior curvature or
checking the whole polygon. Retaining the search path preserves its heading and
turn-radius constraints. A 0.2 change penalty discourages unnecessary left/right
switching, but does not impose a steering slew-rate limit. Necessary S-shaped
maneuvers can still appear, especially for offset or constrained goal headings.

Planner-only validation must use an isolated ROS domain, with no controller or
vehicle-command publisher. Check straight corridors, lateral shifts, a passage
narrower than the padded footprint, and replay poses on the saved map. Compare
path curvature with `tan(max_steering_angle) / wheelbase`; physical driving and
localization/tracking errors still require a separate supervised check.
