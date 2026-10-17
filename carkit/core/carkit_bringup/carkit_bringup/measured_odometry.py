"""Planar dead reckoning from measured forward speed and measured yaw rate.

There are deliberately no motor/steering command inputs. AMCL supplies the
map-to-odom correction; this integrator supplies continuous local motion.
"""

import math


def rotate_vector(rotation, vector):
    """Rotate a sensor vector into the body frame with a unit quaternion."""
    x, y, z, w = rotation
    norm = math.sqrt(sum(value * value for value in rotation))
    if not math.isfinite(norm) or norm < 1e-9 or not all(map(math.isfinite, vector)):
        raise ValueError('Invalid IMU vector or mounting quaternion')
    x, y, z, w = (value / norm for value in (x, y, z, w))
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


class MeasuredOdometry:
    def __init__(self, max_gap=0.25, speed_variance=0.01, yaw_rate_variance=0.0004):
        self.max_gap = max_gap
        self.speed_variance = speed_variance
        self.yaw_rate_variance = yaw_rate_variance
        self.x = self.y = self.yaw = 0.0
        self.covariance = [[0.001, 0.0, 0.0], [0.0, 0.001, 0.0], [0.0, 0.0, 0.001]]
        self.previous = None

    def pause(self):
        """Do not extrapolate across a period without measured motion."""
        if self.previous is not None:
            self.covariance[0][0] += 1.0
            self.covariance[1][1] += 1.0
            self.covariance[2][2] += 0.1
        self.previous = None

    def update(self, stamp, speed, yaw_rate):
        if not all(map(math.isfinite, (stamp, speed, yaw_rate))):
            raise ValueError('Motion measurements must be finite')
        if self.previous is None:
            self.previous = (stamp, speed, yaw_rate)
            return True
        old_stamp, old_speed, old_rate = self.previous
        dt = stamp - old_stamp
        if dt <= 0:
            return False
        if dt > self.max_gap:
            self.pause()
            self.previous = (stamp, speed, yaw_rate)
            return True
        velocity, rate = (speed + old_speed) / 2, (yaw_rate + old_rate) / 2
        angle = rate * dt
        # Exact constant-twist SE(2) integration, stable near a straight line.
        scale = math.sin(angle / 2) / (angle / 2) if abs(angle) > 1e-8 else 1.0
        heading = self.yaw + angle / 2
        dx, dy = velocity * dt * scale * math.cos(heading), velocity * dt * scale * math.sin(heading)
        self.x += dx
        self.y += dy
        self.yaw = math.atan2(math.sin(self.yaw + angle), math.cos(self.yaw + angle))
        # Propagate position/heading coupling plus conservative unmodelled drift.
        f = [[1.0, 0.0, -dy], [0.0, 1.0, dx], [0.0, 0.0, 1.0]]
        p = self.covariance
        fp = [[sum(f[i][k] * p[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        p = [[sum(fp[i][k] * f[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        direction = (math.cos(heading), math.sin(heading))
        for i in range(2):
            for j in range(2):
                p[i][j] += direction[i] * direction[j] * self.speed_variance * dt * dt
            p[i][i] += 0.0025 * dt
        p[2][2] += self.yaw_rate_variance * dt * dt + 0.0001 * dt
        self.covariance = p
        self.previous = (stamp, speed, yaw_rate)
        return True
