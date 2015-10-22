import unittest
import numpy
from state_change_planning import State


class TestStateChangePlanning(unittest.TestCase):

    def test_simpletime(self):
        roll = 5 # radian
        rollSpeed =  1 # radian per second
        rollState = State(roll, rollSpeed)

        rollTarget = State(6,1)

        plan = rollState.planChangeTo(rollTarget).inSeconds(1)
        self.assertEqual(plan.valueAt(0), 5)
        self.assertEqual(plan.valueAt(1), 6)
        self.assertEqual(plan.calculateTargetSpeedForTimePeriod(0,1), 1.0)

    def test_halfsecond(self):
        roll = 5 # radian
        rollSpeed =  1 # radian per second
        rollState = State(roll, rollSpeed)

        rollTarget = State(6,1)

        plan = rollState.planChangeTo(rollTarget).inSeconds(0.5)
        self.assertEqual(plan.valueAt(0), 5)
        self.assertEqual(plan.valueAt(0.5), 6)
        self.assertEqual(plan.calculateTargetSpeedForTimePeriod(0,0.5), 2.0)

    def test_quadratic(self):
        current = State(0, 2)
        target = State(0,-2)
        plan = current.planChangeTo(target).inSeconds(0.25)

        self.assertEqual(plan.valueAt(0), 0)
        self.assertEqual(plan.valueAt(0.25), 0)
        self.assertEqual(plan.valueAt(0.125), 0.5)
        self.assertEqual(plan.calculateTargetSpeedForTimePeriod(0, 0.25), 0)
        self.assertEqual(plan.calculateTargetSpeedForTimePeriod(0, 0.125), 2)
        self.assertEqual(plan.calculateTargetSpeedForTimePeriod(0.125, 0.25), -2)

    def test_cubic(self):
        current = State(10, 30)
        target = State(0,0)
        plan = current.planChangeTo(target).inSeconds(1.5)

        self.assertEqual(plan.valueAt(0), 10)
        self.assertEqual(plan.valueAt(1.5), 0)
        self.assertEqual(plan.calculateTargetSpeedForTimePeriod(0, 1.5), -10/1.5)

if __name__ == '__main__':
    unittest.main()
