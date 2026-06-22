import math
from typing import Optional


def distance_along_path(
    point: tuple[float, float],
    path_points: list[tuple[float, float]],
) -> Optional[float]:
    """Return the along-path distance of a point's closest projection."""
    if len(path_points) < 2:
        return None

    best_distance_sq = math.inf
    best_path_distance = None
    path_distance = 0.0
    for start, end in zip(path_points, path_points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        segment_length_sq = dx * dx + dy * dy
        if segment_length_sq <= 1.0e-12:
            continue

        segment_length = math.sqrt(segment_length_sq)
        fraction = (
            (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
        ) / segment_length_sq
        fraction = max(0.0, min(1.0, fraction))
        projected_x = start[0] + fraction * dx
        projected_y = start[1] + fraction * dy
        distance_sq = (
            (point[0] - projected_x) ** 2
            + (point[1] - projected_y) ** 2
        )
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_path_distance = path_distance + fraction * segment_length
        path_distance += segment_length

    return best_path_distance


def should_stop_before_line(
    remaining_distance_m: float,
    stop_before_distance_m: float,
    stop_line_tolerance_m: float = 0.0,
) -> bool:
    return (
        -max(0.0, stop_line_tolerance_m)
        <= remaining_distance_m
        <= stop_before_distance_m
    )
