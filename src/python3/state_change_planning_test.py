import unittest
import numpy
from state_change_planning import State


class TestStateChangePlanning(unittest.TestCase):

    def test_simpletime(self):
        roll = 5 # radian
        rollSpeed =  1 # radian per second
        rollState = State.by_value_and_speed(roll, rollSpeed)

        rollTarget = State.by_value_and_speed(6, 1)

        plan = rollState.plan_change_to(rollTarget).in_seconds(1)
        self.assertEqual(plan.valueAt(0), 5)
        self.assertEqual(plan.valueAt(1), 6)
        self.assertEqual(plan.calculate_target_speed_for_time_period(0, 1), 1.0)

    def test_halfsecond(self):
        roll = 5 # radian
        rollSpeed =  1 # radian per second
        rollState = State.by_value_and_speed(roll, rollSpeed)

        rollTarget = State.by_value_and_speed(6, 1)

        plan = rollState.plan_change_to(rollTarget).in_seconds(0.5)
        self.assertEqual(plan.valueAt(0), 5)
        self.assertEqual(plan.valueAt(0.5), 6)
        self.assertEqual(plan.calculate_target_speed_for_time_period(0, 0.5), 2.0)

    def test_quadratic(self):
        current = State.by_value_and_speed(0, 2)
        target = State.by_value_and_speed(0, -2)
        plan = current.plan_change_to(target).in_seconds(0.25)

        self.assertEqual(plan.valueAt(0), 0)
        self.assertEqual(plan.valueAt(0.25), 0)
        self.assertEqual(plan.valueAt(0.125), 0.5)
        self.assertEqual(plan.calculate_target_speed_for_time_period(0, 0.25), 0)
        self.assertEqual(plan.calculate_target_speed_for_time_period(0, 0.125), 2)
        self.assertEqual(plan.calculate_target_speed_for_time_period(0.125, 0.25), -2)

    def test_cubic(self):
        current = State.by_value_and_speed(10, 30)
        target = State.by_value_and_speed(0, 0)
        plan = current.plan_change_to(target).in_seconds(1.5)

        self.assertEqual(plan.valueAt(0), 10)
        self.assertEqual(plan.valueAt(1.5), 0)
        self.assertEqual(plan.calculate_target_speed_for_time_period(0, 1.5), -10 / 1.5)

if __name__ == '__main__':
    unittest.main()
