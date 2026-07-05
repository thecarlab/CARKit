# CARKit Vehicle and Nav2 Tuning Reference

## 1. VESC and Chassis Calibration

Configuration file:

- `carkit/vehicle/f1tenth_system/f1tenth_stack/config/vesc.yaml`
- Related code:
  - `carkit/vehicle/f1tenth_system/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp`
  - `carkit/vehicle/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp`

### 1.1 Steering Servo Center `steering_angle_to_servo_offset` (Important)

Current value:

```yaml
steering_angle_to_servo_offset: 0.475
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `steering_angle_to_servo_offset` | `0.475` | `vesc.yaml:10` | `0.35 ~ 0.65` | Servo neutral shifts to the right | Servo neutral shifts to the left |

Center calibration procedure:

1. Test at low speed in an open area.
2. Publish an Ackermann command with `steering_angle = 0`.
3. Check whether the front wheels point straight.
4. If not, adjust `steering_angle_to_servo_offset`; use `0.005 ~ 0.02` per adjustment.
5. Run a low-speed straight-line test and confirm the car no longer drifts left or right.

### 1.2 Speed Gain `speed_to_erpm_gain` (Important)

Current value:

```yaml
speed_to_erpm_gain: 4023.0
```

Effect:

```text
ERPM = speed_to_erpm_gain * speed_mps + speed_to_erpm_offset
odom_speed_mps = (vesc_erpm - speed_to_erpm_offset) / speed_to_erpm_gain
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `speed_to_erpm_gain` | `4023.0` | `vesc.yaml:5` | `3000 ~ 6000` | The same m/s command sends higher ERPM; odom back-calculated speed becomes smaller | The same m/s command sends lower ERPM; odom back-calculated speed becomes larger |

Calibration procedure:

1. Mark a known straight-line distance on the floor, for example `2.5 m`.
2. Record the initial `/odom` value of `pose.pose.position.x`.
3. Drive the car straight for the true distance `d_true`.
4. Record the `/odom` displacement `d_odom`.

Real-car check command:

```bash
ros2 topic echo /odom --field twist.twist.linear.x
```

## 2. Navigation Speed and Path Tracking

Autonomous speed command chain:

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

### 2.1 Nav2 Controller Speed

Configuration file:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

Current value:

```yaml
desired_linear_vel: 1.2
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `desired_linear_vel` | `1.2 m/s` | `nav2_params.yaml:116` | `1 ~ 5 m/s` | Faster cruise speed; more likely to slip or exceed planning/control response | More stable, but may be below the chassis' stable operating speed |

### 2.2 Regulated Pure Pursuit Path Tracking

Current values:

```yaml
min_lookahead_dist: 0.9
max_lookahead_dist: 1.2
lookahead_time: 1.0
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `min_lookahead_dist` | `0.9 m` | `nav2_params.yaml:118` | `0.2 ~ 1.0 m` | Smoother at low speed, but tracks the path less tightly | Tighter low-speed tracking; steering may become busy |
| `max_lookahead_dist` | `1.2 m` | `nav2_params.yaml:119` | `0.8 ~ 3.0 m` | More stable at high speed; may cut into turns earlier | Tracks the path more closely at high speed; may correct frequently |
| `lookahead_time` | `1.0 s` | `nav2_params.yaml:120` | `0.5 ~ 3.0 s` | Lookahead grows more with speed; straights and large turns are smoother | Shorter high-speed lookahead; faster response but higher oscillation risk |

### 2.3 Twist-to-Ackermann Speed Limit

Configuration file:

- `carkit/navigation/carkit_amcl/launch/nav2.launch.py`

Current value:

```python
max_speed: 3.0
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `max_speed` | `3.0 m/s` | `nav2.launch.py:78` | `1 ~ 5.0 m/s` | Allows Nav2 to command higher speed | Caps Nav2's maximum speed |

### 2.4 Nav2 Velocity Smoother

Configuration file:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

Current values:

```yaml
max_velocity: [1.2, 0.0, 1.2]
max_accel: [1.2, 0.0, 1.8]
max_decel: [-3.0, 0.0, -1.8]
```

Array order is `[linear_x, linear_y, angular_z]`:

- Item 1, `linear_x`: forward/backward velocity or acceleration; the main limit for driving and braking.
- Item 2, `linear_y`: lateral motion. Ackermann vehicles cannot move sideways, so keep this at `0.0`.
- Item 3, `angular_z`: yaw velocity or acceleration around the z axis; affects steering response.

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `max_velocity[0]` | `1.2 m/s` | `nav2_params.yaml:329` | `1 ~ 5.0 m/s` | Allows faster forward motion | Limits maximum forward speed |
| `max_velocity[2]` | `1.2 rad/s` | `nav2_params.yaml:329` | `0.5 ~ 2.0 rad/s` | Allows faster yaw response, but turns may become sharper | Gentler turning, but may not keep up with tight curves |
| `max_accel[0]` | `1.2 m/s^2` | `nav2_params.yaml:331` | `0.3 ~ 3.0 m/s^2` | More aggressive launch | Softer launch |
| `max_accel[2]` | `1.8 rad/s^2` | `nav2_params.yaml:331` | `0.5 ~ 3.0 rad/s^2` | Angular velocity changes faster; steering is more responsive | Smoother steering changes, but slower response |
| `max_decel[0]` | `-3.0 m/s^2` | `nav2_params.yaml:332` | `-0.5 ~ -4.0 m/s^2` | Stronger braking | Gentler braking |
| `max_decel[2]` | `-1.8 rad/s^2` | `nav2_params.yaml:332` | `-0.5 ~ -3.0 rad/s^2` | Yaw velocity settles faster, reducing overshoot | Yaw velocity settles more slowly; smoother but may trail |

## 3. AMCL Parameter Tuning

Configuration file:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`

### 3.1 Motion Model Noise

Current values:

```yaml
alpha1: 0.2
alpha2: 0.2
alpha3: 0.8
alpha4: 0.2
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `alpha1` | `0.2` | `nav2_params.yaml:4` | `0.05 ~ 0.5` | More angular spread from in-place rotation or steering | Particles trust rotational odom more |
| `alpha2` | `0.2` | `nav2_params.yaml:5` | `0.05 ~ 0.5` | More angular spread caused by translation | Heading is more stable, but less tolerant of error |
| `alpha3` | `0.8` | `nav2_params.yaml:6` | `0.05 ~ 0.8` | More position spread during straight motion; the robot may appear unsure how far it moved | Trusts odom distance more; less spread |
| `alpha4` | `0.2` | `nav2_params.yaml:7` | `0.05 ~ 0.5` | More position spread caused by turning | Trusts odom more during turns |

The current `alpha3` value, `0.8`, is relatively high and can create large spread during straight driving. If odom has already been calibrated, try:

```yaml
alpha3: 0.2
```

### 3.2 Laser Matching Parameters

Current value:

```yaml
max_beams: 80
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `max_beams` | `80` | `nav2_params.yaml:20` | `40 ~ 120` | Uses more laser beams; matching is more stable but CPU usage increases | Faster, but uses less matching information |

### 3.3 Particle Count and Update Thresholds

Current values:

```yaml
min_particles: 1000
max_particles: 5000
update_min_d: 0.15
update_min_a: 0.15
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `min_particles` | `1000` | `nav2_params.yaml:22` | `500 ~ 2000` | Localization is more stable but uses more CPU | Faster, but may become unstable |
| `max_particles` | `5000` | `nav2_params.yaml:21` | `2000 ~ 8000` | Better recovery from kidnapped robot or large error, but uses more CPU | Faster, but weaker global recovery |
| `update_min_d` | `0.15 m` | `nav2_params.yaml:35` | `0.05 ~ 0.25` | Updates only after moving farther; lower CPU but slower response | Updates more frequently; higher CPU usage |
| `update_min_a` | `0.15 rad` | `nav2_params.yaml:34` | `0.05 ~ 0.3` | Updates only after larger rotations | Updates more frequently while turning |

## 4. Planning, Costmap, and Inflation

Configuration file:

- `carkit/navigation/carkit_amcl/config/nav2_params.yaml`
- Behavior trees:
  - `carkit/navigation/carkit_amcl/behavior_trees/navigate_to_pose_ackermann.xml`
  - `carkit/navigation/carkit_amcl/behavior_trees/navigate_through_poses_ackermann.xml`

### 4.1 Global Costmap Inflation and Obstacle Range

Current values:

```yaml
global_costmap:
  inflation_radius: 0.4
  cost_scaling_factor: 1.5
  obstacle_max_range: 4.0
  raytrace_max_range: 4.5
```

| Parameter | Current Value | Location | Suggested Range | If Increased | If Decreased |
|---|---:|---|---:|---|---|
| `inflation_radius` | `0.4 m` | `nav2_params.yaml:228` | `0.2 ~ 0.8 m` | Global paths keep farther away from walls/obstacles; narrow passages may become blocked | Paths stay closer to walls; riskier |
| `cost_scaling_factor` | `1.5` | `nav2_params.yaml:227` | `1.0 ~ 10.0` | Cost decays faster, so obstacles affect a smaller area | Cost decays more slowly, creating a larger high-cost area |
| `obstacle_max_range` | `4.0 m` | `nav2_params.yaml:209` | `2.0 ~ 6.0` | Farther obstacles are added to the global costmap | Only nearby obstacles are considered |
| `raytrace_max_range` | `4.5 m` | `nav2_params.yaml:211` | Slightly larger than obstacle range | Clears free space farther away | Free-space clearing range is smaller |

Tuning tips:

- To make global paths stay farther from walls, increase `inflation_radius` or decrease `cost_scaling_factor`.
- If `no valid path found` happens often, check whether inflation is too large and blocks the corridor.
