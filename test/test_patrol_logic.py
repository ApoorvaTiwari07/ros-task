import math
import unittest


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TestPatrolLogic(unittest.TestCase):
    def test_angle_normalization(self):
        self.assertTrue(math.isclose(normalize_angle(0.0), 0.0))
        self.assertTrue(
            math.isclose(normalize_angle(math.pi), math.pi, abs_tol=1e-5) or
            math.isclose(normalize_angle(math.pi), -math.pi, abs_tol=1e-5)
        )
        self.assertTrue(
            math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-5) or
            math.isclose(normalize_angle(3 * math.pi), -math.pi, abs_tol=1e-5)
        )

    def test_battery_drain_logic(self):
        # 1% per 10 seconds of motion starting from 100%
        start_pct = 100.0
        seconds_per_pct = 10.0
        motion_time = 25.0  # 25 seconds of motion -> 2.5% drain -> 97.5%
        remaining = max(0.0, start_pct - (motion_time / seconds_per_pct) * 1.0)
        self.assertTrue(math.isclose(remaining, 97.5))

    def test_distance_accumulation(self):
        poses = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        total_dist = 0.0
        for i in range(1, len(poses)):
            dx = poses[i][0] - poses[i-1][0]
            dy = poses[i][1] - poses[i-1][1]
            total_dist += math.hypot(dx, dy)
        self.assertTrue(math.isclose(total_dist, 4.0))

    def test_obstacle_avoidance_turn_decision(self):
        # If left sector distance is greater than right sector distance, robot turns left (positive angular velocity)
        left_dist = 1.2
        right_dist = 0.4
        turn_dir = 1.0 if left_dist >= right_dist else -1.0
        self.assertEqual(turn_dir, 1.0)

        # If right sector distance is greater, robot turns right (negative angular velocity)
        left_dist = 0.3
        right_dist = 0.9
        turn_dir = 1.0 if left_dist >= right_dist else -1.0
        self.assertEqual(turn_dir, -1.0)


if __name__ == '__main__':
    unittest.main()
