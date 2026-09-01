# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Student-owned path tracking algorithm."""

from dataclasses import dataclass

from nav_msgs.msg import Odometry, Path


@dataclass(frozen=True)
class ControlConfig:
    """Limits and vehicle geometry supplied by the ROS node."""

    wheelbase_m: float
    lookahead_m: float
    target_speed_mps: float
    maximum_speed_mps: float
    maximum_steering_rad: float
    goal_tolerance_m: float


@dataclass(frozen=True)
class ControlCommand:
    """Algorithm output before the ROS node applies final safety limits."""

    speed_mps: float = 0.0
    steering_angle_rad: float = 0.0


def compute_command(
    odometry: Odometry,
    path: Path,
    config: ControlConfig,
) -> ControlCommand:
    """Compute one Ackermann command from the current pose and path."""
    # TODO(Intro2AV): Implement pure pursuit, Stanley, MPC, or another path
    # tracker. Do not publish here; return a ControlCommand. The node wrapper
    # handles stale inputs, finite-value checks, clamping, and publication.
    del odometry, path, config
    return ControlCommand()
