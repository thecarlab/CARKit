# CARKit learning annotation: implements the behavior described by this file's package and module.
"""ROS-independent helpers so student algorithms are easy to unit test."""

import math


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


def straight_line_points(start, goal, spacing=0.15):
    """Guided baseline. Replace this with your planner or tune its spacing."""
    distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
    steps = max(1, int(distance / max(0.01, spacing)))
    return [
        (
            start[0] + (goal[0] - start[0]) * index / steps,
            start[1] + (goal[1] - start[1]) * index / steps,
        )
        for index in range(steps + 1)
    ]


def guided_command(position, yaw, path, lookahead=0.55, wheelbase=0.325):
    """Small pure-pursuit baseline with obvious constants for ADA exercises."""
    if not path:
        return 0.0, 0.0
    target = path[-1]
    for candidate in path:
        if math.hypot(candidate[0] - position[0], candidate[1] - position[1]) >= lookahead:
            target = candidate
            break
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    distance_squared = max(dx * dx + dy * dy, 1.0e-6)
    steering = math.atan2(2.0 * wheelbase * local_y, distance_squared)
    remaining = math.hypot(path[-1][0] - position[0], path[-1][1] - position[1])
    speed = 0.0 if remaining < 0.15 else 0.45
    return speed, clamp(steering, -0.34, 0.34)
