import unittest
import numpy
from sensorfusion.attitude_provider import AttitudeProvider
import time


# In order to execute this test, an RTIMULib compatible MPU must be available and configured.
#  This is a check to see if the software is capable of reading values from the hardware.
class TestStateProvider(unittest.TestCase):
    def test_zero(self):
        mock_listener = MockListener(self)
        invalid_counter = InvalidCounterListener()

        attitude_provider = AttitudeProvider()

        attitude_provider.registerListener(mock_listener)
        attitude_provider.registerListener(invalid_counter)

        # give the sensor time to collect values
        time.sleep(0.05)

        attitude_provider.update()

        self.assertEqual(mock_listener.numUpdates, 1)
        self.assertGreater(mock_listener.sumDeltas, 0.05)

        # give the sensor time to collect values
        time.sleep(0.05)

        attitude_provider.update()

        self.assertEqual(mock_listener.numUpdates, 2)
        self.assertGreater(mock_listener.sumDeltas, 0.1)

        # check that all data is present
        self.assertEqual(invalid_counter.accelInvalidCounter, 0)
        self.assertEqual(invalid_counter.gyroInvalidCounter, 0)
        self.assertEqual(invalid_counter.compassInvalidCounter, 0)
        self.assertEqual(invalid_counter.fusionPoseInvalidCounter, 0)

class MockListener:
    def __init__(self, tester):
        self.numUpdates = 0
        self.sumDeltas = 0
        self.tester = tester
        self.name = "MockListener"

    def new_state(self, time_delta, newstate):
        self.numUpdates += 1
        self.sumDeltas += time_delta


class InvalidCounterListener:
    def __init__(self):
        self.accelInvalidCounter = 0
        self.gyroInvalidCounter = 0
        self.compassInvalidCounter = 0
        self.fusionPoseInvalidCounter = 0

        self.name = "InvalidCounterListener"

    def new_state(self, time_delta, newstate):

        # we really do need an orientation estimate
        if not newstate.raw["accelValid"]:
            self.accelInvalidCounter += 1
        if not newstate.raw["gyroValid"]:
            self.gyroInvalidCounter += 1
        if not newstate.raw["compassValid"]:
            self.compassInvalidCounter += 1
        if not newstate.raw["fusionPoseValid"]:
            self.fusionPoseInvalidCounter += 1

if __name__ == '__main__':
    unittest.main()
