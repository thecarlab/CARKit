# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Student-owned global planning algorithm.

Keep ROS subscriptions, validation, timing, and publication in ``planning.py``.
Students implement only the planning policy in this file.
"""

from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry


@dataclass(frozen=True)
class PlanningConfig:
    """Parameters supplied by the ROS node to the planning algorithm."""

    occupancy_threshold: int
    allow_unknown: bool
    inflation_radius_m: float
    waypoint_spacing_m: float
    goal_tolerance_m: float


def compute_path(
    occupancy_grid: OccupancyGrid,
    odometry: Odometry,
    goal: PoseStamped,
    config: PlanningConfig,
) -> list[PoseStamped]:
    """Return a collision-free sequence of poses in the map frame."""
    # TODO(Intro2AV): Implement the global planner here. A typical solution:
    # 1. Convert the start and goal from world coordinates to grid cells.
    # 2. Inflate occupied cells using config.inflation_radius_m.
    # 3. Search the grid with A*, Dijkstra, RRT, or your own method.
    # 4. Convert cells back to PoseStamped waypoints and set orientations.
    # 5. Smooth/resample using config.waypoint_spacing_m.
    del occupancy_grid, odometry, goal, config
    return []
