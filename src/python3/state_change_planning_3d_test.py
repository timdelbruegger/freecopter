import unittest
import numpy
from state_change_planning_3d import State3d


class TestStateChangePlanning3d(unittest.TestCase):

    def test_zero(self):
        zero3d = State3d()
        zeroPlan3d = zero3d.plan_change_to(zero3d).in_seconds(1.0)
        zeroSpeed3d = zeroPlan3d.calculate_target_speed_for_time_period(0.34, 0.7231)

        self.assertEqual(len(zeroSpeed3d), 3)
        self.assertEqual(zeroSpeed3d[0], 0)
        self.assertEqual(zeroSpeed3d[1], 0)
        self.assertEqual(zeroSpeed3d[2], 0)

    def test_one_y(self):
        state3d = State3d()
        target3d = State3d()
        target3d.y.value = 1
        target3d.y.speed = 1
        plan3d = state3d.plan_change_to(target3d).in_seconds(1.0)
        speed3d = plan3d.calculate_target_speed_for_time_period(0.0, 1.0)

        self.assertEqual(len(speed3d), 3)
        self.assertEqual(speed3d[0], 0)
        self.assertEqual(speed3d[1], 1)
        self.assertEqual(speed3d[2], 0)

    def test_one_x(self):
        state3d = State3d()
        target3d = State3d()
        target3d.x.value = 1
        target3d.x.speed = 1
        plan3d = state3d.plan_change_to(target3d).in_seconds(1.0)
        speed3d = plan3d.calculate_target_speed_for_time_period(0.0, 1.0)

        self.assertEqual(len(speed3d), 3)
        self.assertEqual(speed3d[0], 1)
        self.assertEqual(speed3d[1], 0)
        self.assertEqual(speed3d[2], 0)

    def test_one_z(self):
        state3d = State3d()
        target3d = State3d()
        target3d.z.value = 1
        target3d.z.speed = 1
        plan3d = state3d.plan_change_to(target3d).in_seconds(1.0)
        speed3d = plan3d.calculate_target_speed_for_time_period(0.0, 1.0)

        self.assertEqual(len(speed3d), 3)
        self.assertEqual(speed3d[0], 0)
        self.assertEqual(speed3d[1], 0)
        self.assertEqual(speed3d[2], 1)

    def test_xyz(self):
        state3d = State3d()
        target3d = State3d()
        target3d.x.value = 1
        target3d.x.speed = 1
        target3d.y.value = 2
        target3d.y.speed = 2
        target3d.z.value = 3
        target3d.z.speed = 3
        plan3d = state3d.plan_change_to(target3d).in_seconds(1.0)
        speed3d = plan3d.calculate_target_speed_for_time_period(0.0, 1.0)

        self.assertEqual(len(speed3d), 3)
        self.assertEqual(speed3d[0], 1)
        self.assertEqual(speed3d[1], 2)
        self.assertEqual(speed3d[2], 3)

if __name__ == '__main__':
    unittest.main()
