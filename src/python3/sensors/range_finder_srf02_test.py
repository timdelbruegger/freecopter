from sensors.range_finder_srf02 import SRF02
from time import sleep
import datetime
import unittest
import logging


class TestSRF02UltrasonicRangeFinder(unittest.TestCase):

    def setUp(self):
        self.sensor = SRF02()

    def tearDown(self):
        # just wait a little after the last test
        sleep(0.5)
        logging.info("IOErrors during test: {}".format(self.sensor.error_counter))

    def test_single_measurement(self):

        # wait a little
        sleep(0.1)

        self.assertEqual(self.sensor.num_bursts_sent, 0)
        self.sensor._send_burst()
        self.assertEqual(self.sensor.num_bursts_sent, 1)

        sleep(0.1)
        echo = self.sensor._read_echo()
        self.assertIsNotNone(echo)

    def test_update(self):

        # call update multiple times, see that only one burst is fired
        self.sensor.update()
        distance = self.sensor.update()

        self.assertTrue(self.sensor._waiting_for_echo)
        self.assertIsNone(distance)
        self.assertIsNone(self.sensor.distance)
        self.assertIsNotNone(self.sensor.mindistance)
        self.assertEqual(self.sensor.num_bursts_sent, 1)

        # now we wait so that an echo should come
        sleep(0.1)
        distance = self.sensor.update()

        self.assertIsNotNone(distance)
        self.assertIsNotNone(self.sensor.distance)
        self.assertIsNotNone(self.sensor.mindistance)

        # After an Echo is received, we start the next burst immediately
        self.assertEqual(self.sensor.num_bursts_sent, 2)

    def test_20_measurements(self):

        start_time = datetime.datetime.now()

        counter = 0
        while self.sensor.num_bursts_sent < 50:
            distance = self.sensor.update()
            if distance is not None:
                counter += 1

        duration = (datetime.datetime.now()-start_time).total_seconds()
        bursts_per_second = self.sensor.num_bursts_sent / duration
        logging.info("bursts per second: %f" % bursts_per_second)
        logging.info("Executed %d ultrasonic bursts in %f seconds." % (
            self.sensor.num_bursts_sent,duration))

if __name__ == '__main__':
    logging.basicConfig(filename='test_srf02.log', filemode='w', level=logging.DEBUG, format='%(asctime)s %(levelname)s %(name)s %(message)s [%(funcName)s]')
    unittest.main()
