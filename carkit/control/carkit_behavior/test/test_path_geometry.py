import unittest

from carkit_behavior.path_geometry import (
    distance_along_path,
    should_stop_before_line,
)


class PathGeometryTest(unittest.TestCase):
    def test_projects_roadside_sign_onto_straight_path(self):
        path = [(0.0, 0.0), (10.0, 0.0)]
        self.assertAlmostEqual(
            distance_along_path((5.0, 1.0), path),
            5.0,
        )

    def test_measures_along_curved_path(self):
        path = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]
        self.assertAlmostEqual(
            distance_along_path((5.0, 2.0), path),
            6.0,
        )

    def test_rejects_path_without_a_segment(self):
        self.assertIsNone(distance_along_path((0.0, 0.0), []))
        self.assertIsNone(distance_along_path((0.0, 0.0), [(0.0, 0.0)]))

    def test_stops_at_boundary_but_not_before_or_after_window(self):
        self.assertFalse(should_stop_before_line(0.51, 0.5))
        self.assertTrue(should_stop_before_line(0.5, 0.5))
        self.assertTrue(should_stop_before_line(0.0, 0.5))
        self.assertFalse(should_stop_before_line(-0.01, 0.5))
        self.assertTrue(should_stop_before_line(-0.25, 0.5, 0.25))
        self.assertFalse(should_stop_before_line(-0.26, 0.5, 0.25))


if __name__ == "__main__":
    unittest.main()
