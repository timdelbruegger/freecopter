import unittest
from numpy import array
from sensorfusion.height_provider import HeightProvider, correct_ultrasonic_angle
import time
from pyquaternion.quaternion import Quaternion

from state.attitude_state import AttitudeState

from util.definitions import *
from state.attitude_state_test import imu_reading
from state.vehicle_state import VehicleState
from math import isnan, pi
import jsonpickle


#  In order to execute this test, an RTIMULib compatible MPU must be available and configured.
#  This is a check to see if the software is capable of reading values from the hardware.
class TestStateProvider(unittest.TestCase):
    def test_zero(self):
        mock_listener = MockListener(self)
        invalid_counter = InvalidCounterListener()

        height_provider = HeightProvider(RT_IMU_SETTINGS,
                                         ultrasonic_gpio_trigger=GPIO_ULTRASONIC_TRIGGER,
                                         ultrasonic_gpio_echo=GPIO_ULTRASONIC_ECHO)

        attitude_state = AttitudeState(imu_reading)

        # Some warmups
        for x in range(10):
            time.sleep(0.001)
            height_provider.update(attitude_state)

        print("Warmup is done. Now the pressure sensor should be ready...")

        height_provider.registerListener(mock_listener)
        height_provider.registerListener(invalid_counter)

        # give the sensor time to collect values
        time.sleep(0.05)

        height_provider.update(attitude_state)

        self.assertEqual(mock_listener.numUpdates, 1)
        self.assertGreater(mock_listener.sumDeltas, 0.05)

        # give the sensor time to collect values
        time.sleep(0.05)

        height_provider.update(attitude_state)

        self.assertEqual(mock_listener.numUpdates, 2)
        self.assertGreater(mock_listener.sumDeltas, 0.1)

        # check that all data is present
        self.assertEqual(invalid_counter.invalidGpsGroundHeightCounter, 0)
        self.assertEqual(invalid_counter.invalidBarometerGroundHeightCounter, 0)
        self.assertEqual(invalid_counter.pressureInvalidCounter, 0)
        self.assertEqual(invalid_counter.temperatureInvalidCounter, 0)

        self.assertEqual(invalid_counter.invalidElevationSpeedCounter, 0)
        self.assertEqual(invalid_counter.invalidElevationCounter, 0)


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
        self.temperatureInvalidCounter = 0
        self.pressureInvalidCounter = 0
        self.invalidBarometerGroundHeightCounter = 0
        self.invalidGpsGroundHeightCounter = 0
        self.invalidElevationCounter = 0
        self.invalidElevationSpeedCounter = 0

        self.name = "InvalidCounterListener"

    def new_state(self, time_delta, newstate):

        assert(not isnan(time_delta))
        assert(isinstance(newstate, VehicleState))

        print(jsonpickle.encode(newstate))

        if isnan(newstate.temperature):
            self.temperatureInvalidCounter += 1
        if isnan(newstate.air_pressure):
            self.pressureInvalidCounter += 1
        if isnan(newstate.height.height_above_ground):
            self.invalidElevationCounter += 1
        if isnan(newstate.height.ground_height_barometer):
            self.invalidBarometerGroundHeightCounter += 1
        if isnan(newstate.height.ground_height_gps):
            self.invalidGpsGroundHeightCounter += 1
        if isnan(newstate.height.vertical_speed):
            self.invalidElevationSpeedCounter += 1


if __name__ == '__main__':
    unittest.main()
