# CARKit Vehicle and Nav2 Tuning Reference

## 1. VESC and Chassis Calibration

Configuration file:

- `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`
- Related code:
  - `carkit/vehicle/f1tenth_system/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp`
  - `carkit/vehicle/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp`

### 1.1 Steering Servo Center `steering_angle_to_servo_offset` (Important)

| Parameter | Location | Suggested Range | If Increased | If Decreased |
|---|---|---:|---|---|
| `steering_angle_to_servo_offset` | `vesc.yaml:10` | `0.4 ~ 0.55` | Servo neutral shifts to the right | Servo neutral shifts to the left |

Center calibration procedure:

1. Test at low speed in an open area.
2. Publish an Ackermann command with `steering_angle = 0`.
3. Check whether the front wheels point straight.
4. If not, adjust `steering_angle_to_servo_offset`; use `0.005 ~ 0.02` per adjustment.
5. Run a low-speed straight-line test and confirm that the vehicle no longer keeps drifting left or right.

### 1.2 Speed Gain `speed_to_erpm_gain` (Important)

Effect:

```text
ERPM = speed_to_erpm_gain * speed_mps + speed_to_erpm_offset
odom_speed_mps = (vesc_erpm - speed_to_erpm_offset) / speed_to_erpm_gain
```

| Parameter | Location | Suggested Range | If Increased | If Decreased |
|---|---|---:|---|---|
| `speed_to_erpm_gain` | `vesc.yaml:5` | `3800 ~ 4300` | The same m/s command sends higher ERPM; odom back-calculated speed becomes smaller | The same m/s command sends lower ERPM; odom back-calculated speed becomes larger |

Calibration procedure:

1. Mark a known straight-line distance on the floor, for example `2.5m`.
2. Record the initial `/odom` value of `pose.pose.position.x`.
3. Drive the vehicle straight for the true distance `d_true`.
4. Record the `/odom` displacement `d_odom`.

Real-car check command:

```bash
ros2 topic echo /odom --field pose.pose.position.x
```

## 2. Navigation Speed and Path Tracking

### 2.1 Nav2 Controller Speed

Configuration file:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

| Parameter | Location | Suggested Range | If Increased | If Decreased |
|---|---|---:|---|---|
| `desired_linear_vel` | `nav2_params.yaml:116` | `1 ~ 3 m/s` | Faster cruise speed; more likely to slip or exceed planning/control response | More stable, but may be below the chassis' stable operating speed |

### 2.2 Regulated Pure Pursuit Path Tracking

| Parameter | Location | Suggested Range | If Increased | If Decreased |
|---|---|---:|---|---|
| `min_lookahead_dist` | `nav2_params.yaml:118` | `0.2 ~ 1.0 m` | Smoother at low speed, but tracks the path less tightly | Tighter low-speed tracking; steering may become busy |
| `max_lookahead_dist` | `nav2_params.yaml:119` | `0.8 ~ 3.0 m` | More stable at high speed; may cut into turns earlier | Tracks the path more closely at high speed; may correct frequently |
| `lookahead_time` | `nav2_params.yaml:120` | `0.5 ~ 3.0 s` | Lookahead grows more with speed; straights and large turns are smoother | Shorter high-speed lookahead; faster response but higher oscillation risk |

### 2.3 Twist-to-Ackermann Speed Limit

Configuration file:

- `carkit/navigation/carkit_amcl/launch/nav2.launch.py`

| Parameter | Location | Suggested Range | If Increased | If Decreased |
|---|---|---:|---|---|
| `max_speed` | `nav2.launch.py:78` | `3 ~ 5.0 m/s` | Allows Nav2 to command higher speed | Caps Nav2's maximum speed |

### 2.4 Nav2 Velocity Smoother

Configuration file:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

Array order is `[linear_x, linear_y, angular_z]`:

- Item 1, `linear_x`: forward/backward velocity or acceleration; the main limit for driving and braking.
- Item 2, `linear_y`: lateral motion. Ackermann vehicles cannot move sideways, so keep this at `0.0`.
- Item 3, `angular_z`: yaw velocity or acceleration around the z axis; affects steering response.

| Parameter | Location | Suggested Range | If Increased | If Decreased |
|---|---|---:|---|---|
| `max_velocity[0]` | `nav2_params.yaml:329` | `1 ~ 3.0 m/s` | Allows faster forward motion | Limits maximum forward speed |
| `max_velocity[2]` | `nav2_params.yaml:329` | `0.5 ~ 2.0 rad/s` | Allows faster yaw response, but turns may become sharper | Gentler turning, but may not keep up with tight curves |
| `max_accel[0]` | `nav2_params.yaml:331` | `1 ~ 3.0 m/s^2` | More aggressive launch | Softer launch |
| `max_accel[2]` | `nav2_params.yaml:331` | `0.5 ~ 3.0 rad/s^2` | Angular velocity changes faster; steering is more responsive | Smoother steering changes, but slower response |
| `max_decel[0]` | `nav2_params.yaml:332` | `-2 ~ -6.0 m/s^2` | Stronger braking | Gentler braking |
| `max_decel[2]` | `nav2_params.yaml:332` | `-0.5 ~ -3.0 rad/s^2` | Yaw velocity settles faster, reducing overshoot | Yaw velocity settles more slowly; smoother but may trail |

## 3. Planning, Costmap, and Inflation

Configuration file:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`
- Behavior trees:
  - `carkit/navigation/carkit_amcl/behavior_trees/navigate_to_pose_ackermann.xml`
  - `carkit/navigation/carkit_amcl/behavior_trees/navigate_through_poses_ackermann.xml`

### 3.1 Global Costmap Inflation and Obstacle Range

| Parameter | Location | Suggested Range | If Increased | If Decreased |
|---|---|---:|---|---|
| `inflation_radius` | `nav2_params.yaml:228` | `0.2 ~ 1 m` | Global paths keep farther away from walls/obstacles; narrow passages may become blocked | Paths stay closer to walls; riskier |
| `cost_scaling_factor` | `nav2_params.yaml:227` | `0.5 ~ 5.0` | Cost decays faster, so obstacles affect a smaller area | Cost decays more slowly, creating a larger high-cost area |
| `obstacle_max_range` | `nav2_params.yaml:209` | `2.0 ~ 6.0` | Farther obstacles are added to the global costmap | Only nearby obstacles are considered |
| `raytrace_max_range` | `nav2_params.yaml:211` | Slightly larger than obstacle range | Clears free space farther away | Free-space clearing range is smaller |

Tuning tips:

- To make global paths stay farther from walls, increase `inflation_radius` or decrease `cost_scaling_factor`.
- If `no valid path found` happens often, check whether inflation is too large and blocks the corridor.
