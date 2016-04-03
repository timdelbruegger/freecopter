import unittest
from sensorfusion.height_provider import HeightProvider, correct_ultrasonic_angle
import time

from sensorfusion.fusion_master import SensorFusionMaster
import logging


class TestFusionMaster(unittest.TestCase):

    """
    Test to manually check the values that the sensor fusion produces.
    """
    def test_zero(self):


        master = SensorFusionMaster()
        master.update()

        # Some warmups
        for x in range(100):
            master.update()
            time.sleep(0.01)

        print("Warmup is done. Now the pressure sensor should be ready...")

        # give the sensor time to collect values
        time.sleep(0.05)

        for x in range(10000):

            state = master.update()
            self.assertIsNotNone(state.temperature)
            self.assertIsNotNone(state.attitude)
            self.assertIsNotNone(state.height)
            self.assertIsNotNone(state.gps)
            self.assertGreater(state.air_pressure, 100)
            self.assertGreater(state.height.height_above_ground, 0)
            self.assertGreater(state.height.ground_height_barometer, 0)

            logging.info("height above ground: {}".format(state.height.height_above_ground))
            logging.info("vertical speed: {}".format(state.height.vertical_speed))
            logging.info("barometer ground height: {}".format(state.height.ground_height_barometer))

            # TODO: correct axis
            logging.info("yaw, pitch, roll: {}, {}, {}".format(state.attitude.rotation.x, state.attitude.rotation.y, state.attitude.rotation.z))

            time.sleep(0.001)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    unittest.main()
