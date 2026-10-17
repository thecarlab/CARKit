"""Bounded wheel-speed PI correction, in m/s equivalent before ERPM conversion."""
import math


class SpeedFeedback:
    def __init__(self, forward, reverse, kp=0.35, ki=0.6, correction_limit=0.5):
        self.forward, self.reverse = forward, reverse
        self.kp, self.ki, self.limit = kp, ki, correction_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.target = 0.0

    def update(self, target, measured, dt):
        if not all(math.isfinite(v) for v in (target, measured, dt)) or dt <= 0 or dt > 0.25:
            self.reset()
            return 0.0
        target = max(-self.reverse, min(self.forward, target))
        if abs(target) < 1e-6:
            self.reset()
            return 0.0
        if target * self.target <= 0 or abs(target) < abs(self.target) - 0.01:
            self.integral = 0.0
        self.target = target
        error = target - measured
        candidate = max(-self.limit, min(self.limit, self.integral + self.ki * error * dt))
        correction = max(-self.limit, min(self.limit, self.kp * error + candidate))
        low, high = (0.0, self.forward) if target > 0 else (-self.reverse, 0.0)
        raw = target + correction
        output = max(low, min(high, raw))
        # Conditional integration prevents windup at motor or correction limits.
        if (output == raw and abs(self.kp * error + candidate) <= self.limit) or error * candidate < 0:
            self.integral = candidate
        return output
