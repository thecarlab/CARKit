# Vehicle

Vehicle contains the CARKit-facing command interface for the physical platform.

Package:

- `carkit_vehicle_control`

CARKit standard command topic:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)

## Function 1: Controller Only

Use this when a hand controller publishes `ackermann_msgs/AckermannDriveStamped` on `/joy_cmd`, and you want controller commands to go directly to the low-level vehicle command topic through the CARKit mux.

```bash
ros2 launch carkit_vehicle_control controller_only.launch.py \
  vehicle_command_topic:=/ackermann_cmd
```

If your low-level controller expects `/drive`:

```bash
ros2 launch carkit_vehicle_control controller_only.launch.py \
  vehicle_command_topic:=/drive
```

Inputs:

- `/joy_cmd` (`ackermann_msgs/AckermannDriveStamped`)

Output:

- `/ackermann_cmd` by default, or `vehicle_command_topic` (`ackermann_msgs/AckermannDriveStamped`)

Test:

```bash
ros2 topic echo /joy_cmd --once
ros2 topic echo /ackermann_cmd --once
```

## Function 2: Ackermann Command Input

Use this when the autonomy stack publishes `/ackermann_cmd`, or when keyboard control should publish `/ackermann_cmd`.

If the vehicle consumes `/ackermann_cmd` directly:

```bash
ros2 launch carkit_vehicle_control ackermann_input.launch.py
```

If the vehicle consumes `/drive`, relay CARKit commands to `/drive`:

```bash
ros2 launch carkit_vehicle_control ackermann_input.launch.py \
  input_topic:=/ackermann_cmd \
  vehicle_command_topic:=/drive
```

Start keyboard Ackermann control:

```bash
ros2 launch carkit_vehicle_control ackermann_input.launch.py \
  start_keyboard:=true \
  keyboard_topic:=/ackermann_cmd
```

Keyboard controls:

- `w` / `s`: increase or decrease speed
- `a` / `d`: steer left or right
- `x`: center steering
- `space`: stop
- `q`: quit

Inputs:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`) from autonomy or keyboard

Output:

- `/ackermann_cmd` by default, or `vehicle_command_topic` (`ackermann_msgs/AckermannDriveStamped`)

## Notes

CARKit does not know your final hardware transport by default. If your platform needs serial, CAN, VESC, or another actuator protocol, add a vehicle adapter that subscribes to `/ackermann_cmd` or `/drive` and talks to the hardware.
