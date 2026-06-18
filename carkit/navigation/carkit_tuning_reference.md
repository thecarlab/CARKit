# CARKit Vehicle and Nav2 Tuning Reference

This document describes the current CARKit VESC, vehicle-control, AMCL, planner, and costmap parameters. Every value labeled "Current" is taken from the current repository configuration. If the vehicle has not been rebuilt and sourced, the running system may still use an older copy from the `install` directory.

Line numbers refer to the current source revision and may move when configuration files are edited. Use both the file path and parameter name when locating a setting.

Reference:

- F1TENTH odometry calibration guide: <https://f1tenth.readthedocs.io/en/stable/getting_started/driving/drive_calib_odom.html>

The VESC and steering servo operate with ERPM and servo-position values, while the upper control stack uses meters per second and radians. The conversion parameters in `vesc.yaml` must therefore be calibrated for each vehicle.

## 1. VESC and Chassis Calibration

Configuration:

- `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`
- Related implementation:
  - `carkit/vehicle/f1tenth_system/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp`
  - `carkit/vehicle/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp`

### 1.1 Speed Gain `speed_to_erpm_gain` (Important)

Current values:

```yaml
speed_to_erpm_gain: 4023.0
speed_to_erpm_offset: 0.0
```

Conversion:

```text
ERPM = speed_to_erpm_gain * speed_mps + speed_to_erpm_offset
odom_speed_mps = (vesc_erpm - speed_to_erpm_offset) / speed_to_erpm_gain
```

With the current gain:

```text
1.0 m/s -> 4023 ERPM
1.2 m/s -> 4827.6 ERPM
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `speed_to_erpm_gain` | `4023.0` | `vesc.yaml:5` | `3000 - 6000` | Sends more ERPM for the same velocity command; converts the same feedback ERPM into a lower odometry speed | Sends less ERPM for the same velocity command; converts the same feedback ERPM into a higher odometry speed |
| `speed_to_erpm_offset` | `0.0` | `vesc.yaml:6` | Usually keep at `0` | May command the motor even at zero vehicle speed | May cancel small low-speed commands |

Calibration procedure:

1. Mark a straight measured distance, for example `2.5 m`.
2. Record the initial `/odom` `pose.pose.position.x`.
3. Drive the vehicle forward by the measured physical distance `d_true`.
4. Record the odometry displacement `d_odom`.
5. Calculate:

```text
new_gain = old_gain * d_odom / d_true
```

| Observation | Meaning | Adjustment |
|---|---|---|
| Odometry distance is greater than the physical distance | Odometry overestimates motion | Increase `speed_to_erpm_gain` |
| Odometry distance is less than the physical distance | Odometry underestimates motion | Decrease `speed_to_erpm_gain` |
| Forward and reverse results differ substantially | Direction-dependent friction, drivetrain behavior, wheel slip, or VESC behavior | Calibrate primarily for forward autonomous driving and investigate the mechanical asymmetry separately |

Check the running parameters:

```bash
ros2 param get /ackermann_to_vesc_node speed_to_erpm_gain
ros2 param get /vesc_to_odom_node speed_to_erpm_gain
ros2 topic echo /odom --field twist.twist.linear.x
```

### 1.2 Steering Center and Steering Gain (Important)

Current values:

```yaml
steering_angle_to_servo_gain: -1.2135
steering_angle_to_servo_offset: 0.475
servo_min: 0.1
servo_max: 0.9
```

Conversion:

```text
servo_position = steering_angle_to_servo_gain * steering_angle_rad
               + steering_angle_to_servo_offset
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `steering_angle_to_servo_offset` | `0.475` | `vesc.yaml:10` | `0.35 - 0.65` | Moves the steering center toward one side, depending on servo installation | Moves the steering center toward the opposite side |
| `steering_angle_to_servo_gain` | `-1.2135` | `vesc.yaml:9` | About `-0.8 - -1.8`, vehicle dependent | A larger magnitude produces more servo motion for the same steering angle and may hit mechanical limits | A smaller magnitude produces less steering and a larger turning radius |
| `servo_min` | `0.1` | `vesc.yaml:24` | `0.05 - 0.2` | Restricts one steering extreme | May allow the servo to reach a mechanical stop |
| `servo_max` | `0.9` | `vesc.yaml:25` | `0.8 - 0.95` | May allow the servo to reach a mechanical stop | Restricts maximum steering on the other side |

Center calibration:

1. Test at zero speed or at very low speed in an open area.
2. Publish a command with `steering_angle = 0`.
3. Observe whether the front wheels are straight.
4. Adjust `steering_angle_to_servo_offset` in small increments of approximately `0.005 - 0.02`.
5. Perform a low-speed straight-line test and verify that the vehicle no longer drifts consistently left or right.

Example:

```bash
ros2 topic pub --once /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
"{drive: {speed: 0.0, steering_angle: 0.0}}"
```

### 1.3 VESC ERPM Limits

Current values:

```yaml
speed_min: -23250.0
speed_max: 23250.0
```

These values are ERPM limits, not meters per second. With the current gain:

```text
23250 ERPM / 4023 ~= 5.78 m/s
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `speed_max` | `23250.0` | `vesc.yaml:20` | Commonly `4000 - 12000`, depending on the test area | Allows a higher motor speed and increases risk | Reduces the final hardware command limit |
| `speed_min` | `-23250.0` | `vesc.yaml:19` | Commonly `-5000 - 0` | Allows faster reverse motion | Restricts reverse motion; `0` disables reverse motor-speed commands |

Nav2 has its own velocity limits. The VESC ERPM limits should be treated as final low-level protection rather than the normal way to select navigation speed.

## 2. Vehicle Speed, Acceleration, and Angular Motion

Autonomous control chain:

```text
Nav2 /cmd_vel
  -> twist_to_ackermann
  -> /drive
  -> ackermann_mux
  -> /ackermann_cmd
  -> ackermann_to_vesc_node
  -> throttle_interpolator
  -> /commands/motor/speed
  -> vesc_driver_node
```

Manual control chain:

```text
/joy
  -> joy_teleop
  -> /teleop
  -> ackermann_mux
  -> /ackermann_cmd
  -> VESC
```

### 2.1 Nav2 Controller Velocity

Configuration:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

Current values:

```yaml
controller_frequency: 20.0
desired_linear_vel: 1.2
min_approach_linear_velocity: 0.8
regulated_linear_scaling_min_speed: 0.8
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `controller_frequency` | `20.0 Hz` | `nav2_params.yaml:94` | `10 - 30 Hz` | Updates control more frequently but increases CPU load | Reduces CPU load but makes path tracking less responsive |
| `desired_linear_vel` | `1.2 m/s` | `nav2_params.yaml:116` | `1 - 5 m/s` | Increases cruise speed, wheel slip risk, and controller/planner workload | Improves stability but may enter a speed range the chassis cannot maintain |
| `min_approach_linear_velocity` | `0.8 m/s` | `nav2_params.yaml:124` | `0.8 - 3 m/s` | Avoids unstable very-low-speed motion but may overshoot the goal | Approaches the goal more gently but may cause low-speed stutter or motor stall |
| `regulated_linear_scaling_min_speed` | `0.8 m/s` | `nav2_params.yaml:133` | `0.8 - 3 m/s` | Keeps more speed in tight curves | Allows greater slowing in curves but may enter the unstable low-speed range |

Current safety-related settings:

```yaml
use_collision_detection: false
use_cost_regulated_linear_velocity_scaling: false
```

| Parameter | Current | Location | Recommendation | Effect |
|---|---|---|---|---|
| `use_collision_detection` | `false` | `nav2_params.yaml:127` | It may be disabled during isolated tuning; enable it for normal obstacle-aware navigation | Predicts potential collisions and slows or stops the vehicle |
| `use_cost_regulated_linear_velocity_scaling` | `false` | `nav2_params.yaml:131` | Enable it when the vehicle should slow near high-cost regions | Reduces speed when the local costmap cost is high |

### 2.2 Twist-to-Ackermann Limits

Configuration:

- `carkit/navigation/carkit_amcl/launch/nav2.launch.py`

Current values:

```python
max_speed: 1.5
max_reverse_speed: 0.3
max_steering_angle: 0.27
min_speed_for_steering: 0.05
wheelbase: 0.25
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `max_speed` | `1.5 m/s` | `nav2.launch.py:72` | `1 - 5.0 m/s` | Allows a higher Nav2 forward command | Caps the final Nav2 forward command |
| `max_reverse_speed` | `0.3 m/s` | `nav2.launch.py:73` | `0 - 0.5 m/s` | Allows faster reverse motion | Restricts reverse motion; `0` effectively disables it |
| `max_steering_angle` | `0.27 rad` | `nav2.launch.py:74` | `0.15 - 0.45 rad` | Allows tighter commanded turns | Increases commanded turning radius |
| `min_speed_for_steering` | `0.05 m/s` | `nav2.launch.py:75` | `0.02 - 0.15 m/s` | A larger threshold suppresses steering calculation at more low-speed commands | Calculates steering at lower speeds, which can produce large steering angles |
| `wheelbase` | `0.25 m` | `nav2.launch.py:71` | Match the measured wheelbase | Requires more steering angle for the same yaw-rate command | Requires less steering angle for the same yaw-rate command |

### 2.3 Nav2 Velocity Smoother

Configuration:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

Current values:

```yaml
smoothing_frequency: 20.0
max_velocity: [1.8, 0.0, 1.2]
min_velocity: [-0.3, 0.0, -1.2]
max_accel: [1.2, 0.0, 1.8]
max_decel: [-1.5, 0.0, -1.8]
velocity_timeout: 1.0
```

The array order is `[linear_x, linear_y, angular_z]`.

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `smoothing_frequency` | `20.0 Hz` | `nav2_params.yaml:304` | `10 - 50 Hz` | Produces finer updates with more CPU work | Produces coarser updates |
| `max_velocity[0]` | `1.8 m/s` | `nav2_params.yaml:307` | `1 - 5.0 m/s` | Permits faster forward motion | Reduces the forward-speed ceiling |
| `min_velocity[0]` | `-0.3 m/s` | `nav2_params.yaml:308` | `-0.5 - 0` | A more negative value permits faster reverse motion | Restricts reverse motion |
| `max_accel[0]` | `1.2 m/s^2` | `nav2_params.yaml:309` | `0.3 - 3.0 m/s^2` | Produces a more aggressive launch | Produces a smoother, slower launch |
| `max_decel[0]` | `-1.5 m/s^2` | `nav2_params.yaml:310` | `-0.5 - -4.0 m/s^2` | A larger magnitude produces stronger braking | A smaller magnitude produces gentler braking |
| `velocity_timeout` | `1.0 s` | `nav2_params.yaml:314` | `0.2 - 2.0 s` | Keeps the last command valid longer | Clears stale commands sooner |

### 2.4 VESC Throttle Interpolator

Configuration:

- `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`

Current values:

```yaml
max_acceleration: 2.5
throttle_smoother_rate: 75.0
max_servo_speed: 3.2
servo_smoother_rate: 75.0
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `max_acceleration` | `2.5 m/s^2` | `vesc.yaml:55` | `0.5 - 4.0 m/s^2` | Responds faster but may cause jerky launches or wheel slip | Produces smoother starts but slower response |
| `throttle_smoother_rate` | `75 Hz` | `vesc.yaml:56` | `20 - 100 Hz` | Produces finer command steps with slightly more CPU work | Produces coarser output steps |
| `max_servo_speed` | `3.2 rad/s` | `vesc.yaml:58` | `1.0 - 6.0 rad/s` | Makes steering respond faster | Smooths steering but adds response delay |
| `servo_smoother_rate` | `75 Hz` | `vesc.yaml:59` | `20 - 100 Hz` | Produces finer servo updates | Produces coarser servo updates |

## 3. AMCL Tuning

Configuration:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

### 3.1 Motion Model Noise `alpha1 - alpha5`

Current values:

```yaml
alpha1: 0.2
alpha2: 0.2
alpha3: 0.8
alpha4: 0.2
alpha5: 0.2
robot_model_type: nav2_amcl::DifferentialMotionModel
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `alpha1` | `0.2` | `nav2_params.yaml:4` | `0.05 - 0.5` | Increases rotational particle spread caused by rotation | Makes AMCL trust rotational odometry more |
| `alpha2` | `0.2` | `nav2_params.yaml:5` | `0.05 - 0.5` | Increases rotational spread caused by translation | Keeps orientation tighter but reduces tolerance to odometry error |
| `alpha3` | `0.8` | `nav2_params.yaml:6` | `0.05 - 0.8` | Increases translational spread during straight motion | Makes AMCL trust odometry distance more |
| `alpha4` | `0.2` | `nav2_params.yaml:7` | `0.05 - 0.5` | Increases translational spread caused by rotation | Makes AMCL trust odometry more while turning |
| `alpha5` | `0.2` | `nav2_params.yaml:8` | `0.05 - 0.5` | Increases lateral noise; its effect is limited with the differential model | Reduces lateral spread |
| `robot_model_type` | `DifferentialMotionModel` | `nav2_params.yaml:29` | Select the model that matches the chassis | A mismatched model propagates particles differently from the real vehicle | Not a numeric parameter |

If the particle cloud and scan move behind the vehicle during forward/reverse testing, first inspect:

```yaml
alpha3
speed_to_erpm_gain
odom_speed_sign
```

The current `alpha3: 0.8` is relatively high. After odometry calibration, a useful first test is:

```yaml
alpha3: 0.2
```

### 3.2 Laser Matching Parameters

Current values:

```yaml
laser_model_type: likelihood_field
sigma_hit: 0.2
z_hit: 0.8
z_rand: 0.05
laser_likelihood_max_dist: 2.0
max_beams: 80
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `laser_model_type` | `likelihood_field` | `nav2_params.yaml:19` | Usually `likelihood_field` | Not a numeric parameter | Not a numeric parameter |
| `sigma_hit` | `0.2` | `nav2_params.yaml:31` | `0.1 - 0.5` | Makes matching more tolerant of map and scan error | Makes matching stricter and easier to lose |
| `z_hit` | `0.8` | `nav2_params.yaml:36` | `0.6 - 0.95` | Gives more weight to map-consistent scan hits | Gives less weight to map matching |
| `z_rand` | `0.05` | `nav2_params.yaml:38` | `0.02 - 0.3` | Tolerates more random readings and dynamic obstacles | Becomes more sensitive to outliers |
| `laser_likelihood_max_dist` | `2.0 m` | `nav2_params.yaml:16` | `0.5 - 3.0 m` | Extends the distance-field influence and makes matching more tolerant | Restricts matching to points closer to mapped obstacles |
| `max_beams` | `80` | `nav2_params.yaml:20` | `40 - 120` | Uses more scan information but increases CPU load | Reduces CPU load but uses less matching information |

### 3.3 Particle Count and Update Thresholds

Current values:

```yaml
min_particles: 1000
max_particles: 5000
update_min_d: 0.15
update_min_a: 0.15
resample_interval: 1
transform_tolerance: 0.5
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `min_particles` | `1000` | `nav2_params.yaml:22` | `500 - 2000` | Improves robustness but consumes more CPU | Reduces CPU usage but may weaken localization |
| `max_particles` | `5000` | `nav2_params.yaml:21` | `2000 - 8000` | Improves recovery from large errors but consumes more CPU | Reduces recovery capability |
| `update_min_d` | `0.15 m` | `nav2_params.yaml:35` | `0.05 - 0.25 m` | Requires more translation before an AMCL update | Updates more frequently and increases CPU usage |
| `update_min_a` | `0.15 rad` | `nav2_params.yaml:34` | `0.05 - 0.3 rad` | Requires more rotation before an AMCL update | Updates more frequently while turning |
| `resample_interval` | `1` | `nav2_params.yaml:28` | `1 - 3` | Larger values resample less often | Smaller values react faster but may accelerate particle depletion |
| `transform_tolerance` | `0.5 s` | `nav2_params.yaml:33` | `0.2 - 1.0 s` | Tolerates more TF delay | Enforces stricter timing and may expose timestamp problems |

## 4. Planning, Costmap Inflation, and Local Window

Configuration:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`
- Behavior trees:
  - `carkit/navigation/carkit_amcl/behavior_trees/navigate_to_pose_ackermann.xml`
  - `carkit/navigation/carkit_amcl/behavior_trees/navigate_through_poses_ackermann.xml`

### 4.1 Global Costmap Inflation

Current values:

```yaml
global_costmap:
  resolution: 0.05
  track_unknown_space: true
  inflation_radius: 0.3
  cost_scaling_factor: 3.0
  obstacle_max_range: 4.0
  raytrace_max_range: 4.5
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `resolution` | `0.05 m` | `nav2_params.yaml:181` | `0.03 - 0.10 m` | A larger value makes global planning coarser and faster | A smaller value provides finer paths with greater computation |
| `inflation_radius` | `0.3 m` | `nav2_params.yaml:206` | `0.2 - 0.8 m` | Keeps global paths farther from obstacles but may close narrow corridors | Allows paths closer to walls |
| `cost_scaling_factor` | `3.0` | `nav2_params.yaml:205` | `1.0 - 10.0` | Makes inflated cost decay faster with distance | Makes high cost extend farther from obstacles |
| `track_unknown_space` | `true` | `nav2_params.yaml:182` | `true` or `false` | This boolean does not have a numeric increase; `true` preserves unknown-space information | `false` removes unknown-space tracking and may make unexplored areas less distinguishable |
| `obstacle_max_range` | `4.0 m` | `nav2_params.yaml:198` | `2.0 - 6.0 m` | Adds farther scan obstacles to the global costmap | Only marks closer obstacles |
| `raytrace_max_range` | `4.5 m` | `nav2_params.yaml:200` | Slightly greater than obstacle range | Clears free space farther away | Leaves a smaller clearing region and may retain stale obstacles |

To move the global path farther from walls, increase `inflation_radius` or decrease `cost_scaling_factor`. If planning frequently reports `no valid path found`, verify that inflation has not closed the available corridor.

### 4.2 Local Obstacle Inflation

Current values:

```yaml
local_costmap:
  width: 4
  height: 4
  resolution: 0.05
  inflation_radius: 0.15
  cost_scaling_factor: 3.0
  obstacle_max_range: 3.0
  raytrace_max_range: 3.5
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `inflation_radius` | `0.15 m` | `nav2_params.yaml:170` | `0.1 - 0.5 m` | Makes local avoidance more conservative but may block narrow spaces | Allows the vehicle closer to obstacles |
| `cost_scaling_factor` | `3.0` | `nav2_params.yaml:169` | `1.0 - 10.0` | Makes cost decay faster and reduces the effective high-cost region | Extends elevated cost farther from obstacles |
| `obstacle_max_range` | `3.0 m` | `nav2_params.yaml:162` | `2.0 - 5.0 m` | Marks obstacles farther ahead | Only marks closer obstacles |
| `raytrace_max_range` | `3.5 m` | `nav2_params.yaml:164` | Usually `0.3 - 1.0 m` above obstacle range | Clears farther free space | Makes stale obstacle traces more likely |

### 4.3 Local Costmap Window

Current values:

```yaml
width: 4
height: 4
resolution: 0.05
update_frequency: 10.0
publish_frequency: 5.0
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `width` | `4 m` | `nav2_params.yaml:147` | `3 - 8 m` | Looks farther ahead/behind but increases CPU and memory use | Reduces computation but shortens obstacle awareness |
| `height` | `4 m` | `nav2_params.yaml:148` | `3 - 8 m` | Covers a wider lateral region but increases CPU and memory use | Reduces the local obstacle field |
| `resolution` | `0.05 m` | `nav2_params.yaml:149` | `0.03 - 0.10 m` | A numerically larger resolution is coarser and faster | A smaller resolution is more detailed and more expensive |
| `update_frequency` | `10 Hz` | `nav2_params.yaml:142` | `5 - 15 Hz` | Updates obstacles faster but consumes more CPU | Increases obstacle-update latency |
| `publish_frequency` | `5 Hz` | `nav2_params.yaml:143` | `2 - 10 Hz` | Makes RViz updates more responsive | Reduces visualization and transport load |

For speeds above approximately `1.0 m/s`, a possible next test is:

```yaml
width: 5
height: 5
obstacle_max_range: 4.0
raytrace_max_range: 4.5
```

Do not enlarge the window aggressively if the platform already reports Behavior Tree tick-rate warnings or high CPU usage.

### 4.4 Footprint (Shared by All Vehicles)

Current value:

```yaml
footprint: "[[0.435, 0.135], [0.435, -0.135], [-0.05, -0.135], [-0.05, 0.135]]"
```

| Parameter | Current | Location | Recommended basis | Increasing it | Decreasing it |
|---|---|---|---|---|---|
| `footprint` | Front `0.435 m`, rear `0.05 m`, half-width `0.135 m` | local: `nav2_params.yaml:150`; global: `nav2_params.yaml:183` | Measure the physical vehicle and add a small safety margin | Improves collision protection but increases `starting point in lethal space` and narrow-corridor failures | Makes planning easier but increases collision risk |

If the planner reports:

```text
Starting point in lethal space
```

check the footprint, localization offset, LiDAR transform, and scan points close to the chassis.

### 4.5 Global Planner

Current values:

```yaml
planner_plugins: [GridBased]
plugin: nav2_smac_planner/SmacPlannerHybrid
expected_planner_frequency: 5.0
tolerance: 0.5
allow_unknown: true
max_iterations: 1000000
max_planning_time: 10.0
motion_model_for_search: DUBIN
minimum_turning_radius: 1.0
analytic_expansion_max_length: 3.0
lookup_table_size: 20.0
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `planner_plugins` | `[GridBased]` | `nav2_params.yaml:228` | At least one valid planner ID | More plugins provide more strategies but increase configuration complexity | Removing the only plugin prevents planning |
| `GridBased.plugin` | `SmacPlannerHybrid` | `nav2_params.yaml:230` | A Hybrid-A* planner is appropriate for Ackermann vehicles | Not a numeric parameter | Not a numeric parameter |
| `expected_planner_frequency` | `5.0 Hz` | `nav2_params.yaml:226` | `1 - 10 Hz` | Raises the expected planning rate and CPU requirement | Accepts slower path updates |
| `max_planning_time` | `10.0 s` | `nav2_params.yaml:237` | `2 - 10 s` | Gives long or complex paths more search time but delays failure reporting | Causes difficult plans to terminate earlier |
| `max_iterations` | `1000000` | `nav2_params.yaml:235` | `200000 - 3000000` | Allows more search work but may take longer | Makes complex searches fail earlier |
| `tolerance` | `0.5 m` | `nav2_params.yaml:233` | `0.1 - 1.0 m` | Allows the path to terminate farther from an obstructed exact goal | Requires a more exact endpoint |
| `allow_unknown` | `true` | `nav2_params.yaml:234` | `true` or `false` | `true` permits planning through unknown cells and may increase risk | `false` avoids unknown cells but may reduce connectivity |
| `motion_model_for_search` | `DUBIN` | `nav2_params.yaml:238` | Common for forward-only Ackermann motion | Not a numeric parameter | Not a numeric parameter |
| `minimum_turning_radius` | `1.0 m` | `nav2_params.yaml:243` | Match the measured vehicle minimum radius | Produces wider, more conservative turns | Allows tighter paths that the real vehicle may not follow |
| `analytic_expansion_max_length` | `3.0 m` | `nav2_params.yaml:241` | `1 - 5 m` | Permits a longer analytic connection near the goal | Uses a shorter, more conservative final connection |
| `lookup_table_size` | `20.0 m` | `nav2_params.yaml:249` | `10 - 40 m` | Extends the heuristic lookup region with greater memory/initialization cost | Reduces the lookup region for long searches |

There is no explicit maximum total path length. Long-path failures are usually caused by an unreachable segment, a goal outside the map, a goal inside a high-cost cell, insufficient planning time, or an infeasible turning geometry. A single unreachable waypoint can fail the complete `NavigateThroughPoses` request.

### 4.6 Behavior Tree and Replanning Frequencies

Current value:

```yaml
bt_loop_duration: 50
```

Behavior-tree rates:

```xml
<!-- Single goal -->
<RateController hz="1.0">

<!-- Navigate through poses -->
<RateController hz="0.333">
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `bt_loop_duration` | `50 ms` | `nav2_params.yaml:49` | `20 - 100 ms` | Reduces BT tick frequency and CPU load but slightly delays high-level state handling | Raises tick frequency and CPU load and may trigger tick-rate warnings |
| Single-goal `RateController` | `1.0 Hz` | `navigate_to_pose_ackermann.xml:11` | `0.5 - 2.0 Hz` | Replans more often | Updates the global path less often |
| Multi-pose `RateController` | `0.333 Hz` | `navigate_through_poses_ackermann.xml:12` | `0.2 - 1.0 Hz` | Replans a multi-pose route more often | Reduces CPU usage but reacts more slowly to route changes |

`bt_loop_duration` controls high-level Behavior Tree scheduling. It does not directly control motor speed. Vehicle control output is mainly determined by `controller_frequency` and the downstream command chain.

### 4.7 Waypoint / Nav Through Poses Mode

After selecting `Waypoint / Nav Through Poses Mode` in the RViz `Navigation 2` panel, add several poses with the `Nav2 Goal` tool and choose one of these actions:

| Button | Action | Execution model | Intermediate-pose behavior | Typical use |
|---|---|---|---|---|
| `Start Nav Through Poses` | `/navigate_through_poses` | Plans one continuous path through all supplied poses | Normally passes intermediate poses without executing a waypoint task | Continuous driving and advance planning of the complete route |
| `Start Waypoint Following` | `/follow_waypoints` | Executes each waypoint as a separate navigation goal | May pause or execute a task at every waypoint | Inspection, scheduled stops, and lower per-plan complexity |
| `Cancel Accumulation` | No navigation action | Clears the accumulated poses before they are sent | Does not start navigation | Correcting an incorrectly drawn route |

`Waypoint / Nav Through Poses Mode` is a runtime state of the RViz plugin, not a `nav2_params.yaml` parameter. The current Humble plugin does not persist this mode as the RViz default, so it must be selected manually after Nav2 becomes active.

#### Navigate Through Poses Parameters

Behavior tree:

- `carkit/navigation/carkit_amcl/behavior_trees/navigate_through_poses_ackermann.xml`

Current values:

```xml
<RecoveryNode number_of_retries="6" name="NavigateRecovery">
<RateController hz="0.333">
<RemovePassedGoals radius="0.7"/>
<Wait wait_duration="5"/>
```

| Parameter | Current | Location | Practical range | Increasing it | Decreasing it |
|---|---:|---|---:|---|---|
| `NavigateRecovery.number_of_retries` | `6` | `navigate_through_poses_ackermann.xml:10` | `1 - 6` | Performs more recovery cycles after planning or control failures and takes longer to abort | Aborts sooner but provides fewer opportunities for automatic recovery |
| Multi-pose `RateController` | `0.333 Hz` | `navigate_through_poses_ackermann.xml:12` | `0.2 - 1.0 Hz` | Replans more frequently and responds faster to map changes, with greater computation | Saves CPU but reacts more slowly to route and obstacle changes |
| `RemovePassedGoals.radius` | `0.7 m` | `navigate_through_poses_ackermann.xml:15` | `0.3 - 1.0 m` | Marks poses as passed earlier, improving continuity but allowing more corner cutting | Requires the vehicle to pass closer to each pose and may cause slowing or returning |
| Recovery `wait_duration` | `5 s` | `navigate_through_poses_ackermann.xml:33` | `1 - 10 s` | Waits longer for the environment or sensors to recover | Retries sooner but may repeat failures rapidly |

`ComputePathThroughPoses` processes every supplied pose in one planning request. More poses, longer distances, and complex turns increase Hybrid-A* search cost. These planner parameters directly affect multi-pose planning performance:

```yaml
downsample_costmap: false
downsampling_factor: 1
angle_quantization_bins: 72
cache_obstacle_heuristic: false
max_planning_time: 10.0
```

| Parameter | Current | Location | Practical range | Increasing/enabling it | Decreasing/disabling it |
|---|---:|---|---:|---|---|
| `downsample_costmap` | `false` | `nav2_params.yaml:231` | `true` or `false` | Uses a lower-resolution search grid, making long plans faster but coarser | Uses full costmap resolution for finer paths with greater computation |
| `downsampling_factor` | `1` | `nav2_params.yaml:232` | `1 - 4` | A larger factor makes planning faster and coarser and may lose narrow-passage detail | A smaller factor preserves detail but costs more; only applies when downsampling is enabled |
| `angle_quantization_bins` | `72` | `nav2_params.yaml:239` | `32 - 72` | Represents heading more precisely but increases the search state space | Plans faster with coarser heading resolution and potentially less-smooth paths |
| `cache_obstacle_heuristic` | `false` | `nav2_params.yaml:250` | `true` or `false` | May accelerate repeated planning toward similar goals, while using memory and assuming a relatively stable environment | Recomputes the heuristic for each plan and may be slower for repeated requests |
| `max_planning_time` | `10.0 s` | `nav2_params.yaml:237` | `2 - 10 s` | Gives long multi-pose routes more search time but delays failure feedback | Reports failure sooner but may terminate before finding a complex route |

Recommendations for long routes:

1. Use `Start Nav Through Poses` when intermediate stops are not desired.
2. Start with approximately `3 - 6` poses per request and increase only after confirming stable performance.
3. Place poses near the center of traversable space, away from walls, map boundaries, and high-cost cells.
4. Use continuous, physically achievable headings around turns. The current `DUBIN` model does not permit reversing.
5. If planning times out, first try `downsample_costmap: true`, `downsampling_factor: 2`, or fewer `angle_quantization_bins`.

#### Waypoint Following Parameters

Configuration:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

Current values:

```yaml
waypoint_follower:
  loop_rate: 20
  stop_on_failure: false
  waypoint_task_executor_plugin: wait_at_waypoint
  wait_at_waypoint:
    enabled: true
    waypoint_pause_duration: 200
```

| Parameter | Current | Location | Practical range | Increasing/enabling it | Decreasing/disabling it |
|---|---:|---|---:|---|---|
| `loop_rate` | `20 Hz` | `nav2_params.yaml:293` | `5 - 50 Hz` | Checks waypoint task state more frequently with slightly more CPU use | Updates task state more slowly with less computation |
| `stop_on_failure` | `false` | `nav2_params.yaml:294` | `true` or `false` | `true` aborts the entire request when any waypoint fails | `false` records the failure and continues to later waypoints |
| `waypoint_task_executor_plugin` | `wait_at_waypoint` | `nav2_params.yaml:295` | Select a plugin for the required task | Another plugin can perform a different action at each waypoint | Removing or misconfiguring it prevents waypoint tasks from running |
| `wait_at_waypoint.enabled` | `true` | `nav2_params.yaml:298` | `true` or `false` | Executes the wait task after reaching each waypoint | Skips the wait task |
| `waypoint_pause_duration` | `200 ms` | `nav2_params.yaml:299` | `0 - 5000 ms` | Waits longer at each waypoint | `0` removes the deliberate wait, although action transitions may still cause a brief pause |

For advance planning of a complete route without intermediate stops, use `Start Nav Through Poses`. Setting `waypoint_pause_duration` to `0` does not convert Waypoint Following into a single continuous multi-pose plan.

## 5. Recommended Tuning Order

### 5.1 Incorrect Straight-Line Odometry

1. Calibrate `speed_to_erpm_gain`.
2. Verify `odom_speed_sign`.
3. Compare `/commands/motor/speed` with `/sensors/core`.
4. Tune AMCL `alpha3` only after the odometry scale is credible.

### 5.2 Excessive Particle Spread or Poor Scan Matching

First test:

```yaml
alpha3: 0.2
sigma_hit: 0.25
z_rand: 0.1
max_beams: 60
```

Change only one or two parameters at a time and repeat the same route.

### 5.3 Planning Failure and Repeated Recovery

Look for:

```text
Starting point in lethal space
no valid path found
Failed to make progress
```

| Message | Check first |
|---|---|
| `Starting point in lethal space` | Footprint, localization, chassis scan points, and costmap inflation |
| `no valid path found` | Goal outside the map, goal inside an obstacle/unknown region, and blocked global inflation |
| `Failed to make progress` | Chassis command chain, odometry, and progress checker |

### 5.4 Vehicle Too Slow or Unstable at Low Speed

Tune these parameters first:

```yaml
desired_linear_vel
max_velocity[0]
max_speed
max_acceleration
```

Do not set every minimum velocity to `1.0 m/s`. The controller still needs a usable approach and stopping behavior near the goal.

## 6. Applying Configuration Changes

If package-share files in `install` are not symlinks, rebuild:

```bash
colcon build --symlink-install --packages-select carkit_amcl carkit_navigation f1tenth_stack
source install/setup.bash
```

For Nav2 parameter changes only:

```bash
colcon build --symlink-install --packages-select carkit_amcl
source install/setup.bash
```

For RViz changes only:

```bash
colcon build --symlink-install --packages-select carkit_navigation
source install/setup.bash
```

For VESC or vehicle-control configuration changes:

```bash
colcon build --symlink-install --packages-select f1tenth_stack
source install/setup.bash
```

Check whether installed files are symlinks:

```bash
ls -l install/carkit_amcl/share/carkit_amcl/config/nav2_params.yaml
ls -l install/f1tenth_stack/share/f1tenth_stack/config/vesc.yaml
```

If the output points to the source file, restarting the launch process is normally sufficient. Otherwise, rebuild to copy the updated resources into `install`.
