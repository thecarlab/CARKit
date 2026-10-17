"""Compatibility import; joystick ownership is shared by every chassis."""
from carkit_human_control.joystick_teleop import JoystickTeleop, main

__all__ = ["JoystickTeleop", "main"]

if __name__ == "__main__":
    main()
