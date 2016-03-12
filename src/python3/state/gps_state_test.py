import unittest
from state.attitude_state import AttitudeState
import gpspy3.gps as gps
from state.gps_state import GPSState

gps_reading = gps.GPSData()
gps_reading.fix.altitude = 1.0
gps_reading.fix.climb = 2.0
gps_reading.fix.epc = 3.0
gps_reading.fix.epd = 4.0
gps_reading.fix.eps = 5.0
gps_reading.fix.ept = 6.0
gps_reading.fix.epv = 7.0
gps_reading.fix.epx = 8.0
gps_reading.fix.epy = 9.0
gps_reading.fix.latitude = 10.0
gps_reading.fix.longitude = 22.0
gps_reading.fix.speed = 29.0
gps_reading.fix.time = 123.0


# We will just try to create AttitudeStates from the sample IMU reading
class TestGpsState(unittest.TestCase):

    def test_sample(self):

        gps_state = GPSState(gps_reading)

        # raw state is just a copy
        self.assertEqual(gps_state.rawfix, gps_reading.fix)

        self.assertEqual(gps_state.speed, gps_reading.fix.speed)
        self.assertEqual(gps_state.climb, gps_reading.fix.climb)
        self.assertEqual(gps_state.latitude, gps_reading.fix.latitude)
        self.assertEqual(gps_state.longitude, gps_reading.fix.longitude)
        self.assertEqual(gps_state.time, gps_reading.fix.time)
        self.assertEqual(gps_state.speed_error, gps_reading.fix.eps)
        self.assertEqual(gps_state.longitude_error, gps_reading.fix.epx)
        self.assertEqual(gps_state.latitude_error, gps_reading.fix.epy)
        self.assertEqual(gps_state.altitude_error, gps_reading.fix.epv)
        self.assertEqual(gps_state.climb_error, gps_reading.fix.epc)



        # TODO: tests for readError, also if we have no fix!


if __name__ == '__main__':
    unittest.main()
