import math
import unittest

from carkit_student_algorithms.math_utils import guided_command, straight_line_points


class TestAlgorithms(unittest.TestCase):
    def test_straight_line_includes_endpoints(self):
        points = straight_line_points((0.0, 0.0), (1.0, 0.0), spacing=0.2)
        self.assertEqual(points[0], (0.0, 0.0))
        self.assertEqual(points[-1], (1.0, 0.0))

    def test_guided_controller_stops_at_goal(self):
        speed, steering = guided_command((0.0, 0.0), 0.0, [(0.05, 0.0)])
        self.assertEqual(speed, 0.0)
        self.assertEqual(steering, 0.0)

    def test_guided_controller_turns_toward_path(self):
        speed, steering = guided_command((0.0, 0.0), 0.0, [(1.0, 1.0)])
        self.assertGreater(speed, 0.0)
        self.assertLessEqual(steering, math.radians(20.0))
        self.assertGreater(steering, 0.0)
