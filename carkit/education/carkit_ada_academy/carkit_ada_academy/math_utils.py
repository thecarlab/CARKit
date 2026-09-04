# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Small ROS-independent functions intended for classroom modification."""

import math


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


def straight_line_points(start, goal, spacing=0.15):
    """Create a visible baseline path students can modify incrementally."""
    distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
    steps = max(1, int(distance / max(0.01, spacing)))
    return [
        (
            start[0] + (goal[0] - start[0]) * index / steps,
            start[1] + (goal[1] - start[1]) * index / steps,
        )
        for index in range(steps + 1)
    ]


def pure_pursuit_command(
    position,
    yaw,
    path,
    lookahead=0.55,
    wheelbase=0.325,
    speed=0.45,
):
    """Return a stable pure-pursuit command for the unconsumed path ahead."""
    if not path:
        return 0.0, 0.0

    # Searching from path[0] can select an already-passed waypoint once it is
    # a lookahead distance behind the car, causing alternating steering.
    nearest_index = min(
        range(len(path)),
        key=lambda index: math.hypot(
            path[index][0] - position[0],
            path[index][1] - position[1],
        ),
    )
    target = path[-1]
    distance_ahead = 0.0
    previous = path[nearest_index]
    for candidate in path[nearest_index + 1:]:
        distance_ahead += math.hypot(
            candidate[0] - previous[0],
            candidate[1] - previous[1],
        )
        if distance_ahead >= max(0.05, lookahead):
            target = candidate
            break
        previous = candidate
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    distance_squared = max(dx * dx + dy * dy, 1.0e-6)
    steering = math.atan2(2.0 * wheelbase * local_y, distance_squared)
    remaining = math.hypot(
        path[-1][0] - position[0],
        path[-1][1] - position[1],
    )
    if remaining < 0.12:
        commanded_speed = 0.0
    elif remaining < 0.60:
        commanded_speed = max(0.18, speed * remaining / 0.60)
    else:
        commanded_speed = speed

    # Reduce speed in tight turns to avoid lateral slip.
    commanded_speed *= max(0.55, 1.0 - abs(steering) / 0.68)
    return commanded_speed, clamp(steering, -0.34, 0.34)


def keep_detection(class_name, confidence, minimum_confidence=0.35):
    """ADA exercise seam for class and confidence filtering."""
    del class_name
    return confidence >= minimum_confidence
