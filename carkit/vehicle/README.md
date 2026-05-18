# Vehicle

This module is reserved for vehicle-specific adapters.

Current command output from CARKit:

- `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)

TODO: add a vehicle adapter here if the platform needs serial, CAN, VESC, or custom actuator translation.

Test the current command stream:

```bash
ros2 topic echo /ackermann_cmd
```
