from sensors.pressure_sensor import PressureTemperatureSensor
from multiprocessing import Queue, Value
import threading
import time

from util.definitions import RT_IMU_SETTINGS

import unittest

from util.definitions import GPIO_ULTRASONIC_ECHO, GPIO_ULTRASONIC_TRIGGER


# In order to execute this test, a pressure sensor has to be connected to the RPI via i2c.
class PressureSensorTest(unittest.TestCase):

    def test_values_good(self):
        sensor = PressureTemperatureSensor(RT_IMU_SETTINGS)

        # Some warmups
        for x in range(10):
            time.sleep(0.01)
            reading = sensor.read()

        print("temperature: ", reading.temperature)
        print("pressure: ", reading.pressure)
        print("height above zero: ", reading.height_above_sea)
        print("height above zero expected error: ", reading.height_above_sea_error)

        self.assertIsNotNone(reading)
        self.assertIsNotNone(reading.pressure)
        self.assertGreater(reading.pressure, 0)
        self.assertGreater(reading.temperature, 0)
        self.assertGreater(reading.height_above_sea, 0)
        self.assertGreater(reading.height_above_sea_error, 0)


if __name__ == '__main__':
    unittest.main()
