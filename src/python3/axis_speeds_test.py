import unittest
import numpy
from axis_speeds import AxisSpeeds

class TestAxisSpeeds(unittest.TestCase):

    def test_zero(self):
        speeds = AxisSpeeds()
        speeds.xAxisRotationSpeed = 0 # roll axis
        speeds.yAxisRotationSpeed = 0 # yaw axis
        speeds.zAxisRotationSpeed = 0 # pitch axis
        speeds.verticalSpeed = 0 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.zeros(4)).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.zeros(4)).all())

    def test_roll(self):
        speeds = AxisSpeeds()
        speeds.xAxisRotationSpeed = 1 # roll axis
        speeds.yAxisRotationSpeed = 0 # yaw axis
        speeds.zAxisRotationSpeed = 0 # pitch axis
        speeds.verticalSpeed = 0 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.array([1, 0, 0, 0])).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.array([0.25, -0.25, 0.25, -0.25])).all())

    def test_yaw(self):
        speeds = AxisSpeeds()
        speeds.xAxisRotationSpeed = 0 # roll axis
        speeds.yAxisRotationSpeed = 1 # yaw axis
        speeds.zAxisRotationSpeed = 0 # pitch axis
        speeds.verticalSpeed = 0 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.array([0, 1, 0, 0])).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.array([0.25, -0.25, -0.25, 0.25])).all())

    def test_pitch(self):
        speeds = AxisSpeeds()
        speeds.xAxisRotationSpeed = 0 # roll axis
        speeds.yAxisRotationSpeed = 0 # yaw axis
        speeds.zAxisRotationSpeed = 1 # pitch axis
        speeds.verticalSpeed = 0 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.array([0, 0, 1, 0])).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.array([0.25, 0.25, -0.25, -0.25])).all())

    def test_up(self):
        speeds = AxisSpeeds()
        speeds.xAxisRotationSpeed = 0 # roll axis
        speeds.yAxisRotationSpeed = 0 # yaw axis
        speeds.zAxisRotationSpeed = 0 # pitch axis
        speeds.verticalSpeed = 1 # up

        self.assertEqual(len(speeds.asArray()), 4)
        self.assertTrue((speeds.asArray() == numpy.array([0, 0, 0, 1])).all())

        # expected Result: zero matrix
        self.assertEqual(len(speeds.toMotorSignals()), 4)
        self.assertTrue((speeds.toMotorSignals() == numpy.array([0.25, 0.25, 0.25, 0.25])).all())

if __name__ == '__main__':
    unittest.main()
