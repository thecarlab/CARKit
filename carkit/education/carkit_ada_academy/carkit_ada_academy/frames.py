"""Coordinate conversion shared by the guided planner and controller."""

import math

from rclpy.time import Time


def transform_points(buffer, points, source_frame, target_frame, stamp):
    """Transform planar points at the odometry timestamp; never assume identity."""
    if not source_frame or not target_frame:
        raise ValueError("Both source and target frames are required")
    if not all(math.isfinite(v) for point in points for v in point):
        raise ValueError("Path coordinates must be finite")
    if source_frame == target_frame:
        return list(points)
    transform = buffer.lookup_transform(
        target_frame, source_frame, Time.from_msg(stamp)).transform
    q, t = transform.rotation, transform.translation
    values = (q.x, q.y, q.z, q.w, t.x, t.y, t.z)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Transform must be finite")
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        raise ValueError("Transform quaternion must be nonzero")
    x, y, z, w = (q.x/norm, q.y/norm, q.z/norm, q.w/norm)
    return [
        (t.x + (1 - 2*(y*y + z*z))*px + 2*(x*y - z*w)*py,
         t.y + 2*(x*y + z*w)*px + (1 - 2*(x*x + z*z))*py)
        for px, py in points
    ]


def odometry_fresh(message, received_at, now, timeout):
    return message is not None and received_at is not None and 0 <= now - received_at <= timeout
