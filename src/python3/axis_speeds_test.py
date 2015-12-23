import unittest
import numpy
from axis_speeds import AxisSpeeds

class TestAxisSpeeds(unittest.TestCase):

    def test_zero(self):
        speeds = AxisSpeeds()
        speeds.axis_rotation_speeds[0] = 0 # roll axis
        speeds.axis_rotation_speeds[1] = 0 # yaw axis
        speeds.axis_rotation_speeds[2] = 0 # pitch axis
        speeds.vertical_speed = 0 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.zeros(4)).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.zeros(4)).all())

    def test_roll(self):
        speeds = AxisSpeeds()
        speeds.axis_rotation_speeds[0] = 1 # roll axis
        speeds.axis_rotation_speeds[1] = 0 # yaw axis
        speeds.axis_rotation_speeds[2] = 0 # pitch axis
        speeds.vertical_speed = 0 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.array([1, 0, 0, 0])).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.array([0.25, -0.25, 0.25, -0.25])).all())

    def test_yaw(self):
        speeds = AxisSpeeds()
        speeds.axis_rotation_speeds[0] = 0 # roll axis
        speeds.axis_rotation_speeds[1] = 1 # yaw axis
        speeds.axis_rotation_speeds[2] = 0 # pitch axis
        speeds.vertical_speed = 0 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.array([0, 1, 0, 0])).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.array([0.25, -0.25, -0.25, 0.25])).all())

    def test_pitch(self):
        speeds = AxisSpeeds()
        speeds.axis_rotation_speeds[0] = 0 # roll axis
        speeds.axis_rotation_speeds[1] = 0 # yaw axis
        speeds.axis_rotation_speeds[2] = 1 # pitch axis
        speeds.vertical_speed = 0 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.array([0, 0, 1, 0])).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.array([0.25, 0.25, -0.25, -0.25])).all())

    def test_up(self):
        speeds = AxisSpeeds()
        speeds.axis_rotation_speeds[0] = 0 # roll axis
        speeds.axis_rotation_speeds[1] = 0 # yaw axis
        speeds.axis_rotation_speeds[2] = 0 # pitch axis
        speeds.vertical_speed = 1 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.array([0, 0, 0, 1])).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.array([0.25, 0.25, 0.25, 0.25])).all())

if __name__ == '__main__':
    unittest.main()
