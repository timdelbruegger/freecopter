from sensors.range_finder import UltrasonicRangeFinder
from multiprocessing import Queue, Value
import threading

import logging

import unittest

from util.definitions import GPIO_ULTRASONIC_ECHO, GPIO_ULTRASONIC_TRIGGER


# In order to execute this test, an ultrasonic range finder has to be connected to the RPI GPIO ports above.
# We don't test the multiprocessing here, but use the same data structures
class TestStateProvider(unittest.TestCase):

    # just sets the stop flag so that the range finder loop will exit
    def stop_range_finder(self, stop_flag):
        print("Ultrasonic sensor should stop now.")
        stop_flag.value = True

    def test_values_good(self):
        queue = Queue(1000)
        stop_flag = Value('i', False)
        rangefinder = UltrasonicRangeFinder(GPIO_ULTRASONIC_TRIGGER, GPIO_ULTRASONIC_ECHO)

        # this should give us around 50 measurements
        t = threading.Timer(0.50, self.stop_range_finder, args=(stop_flag,))
        t.start()

        # this will loop until the timer kicks in
        rangefinder.measure(queue, stop_flag, 0)

        # last value in queue should be equal to current distance approximation
        last_distance = None
        count = 0
        while not queue.empty():
            count += 1
            last_distance = queue.get_nowait()
            self.assertGreater(last_distance.value, 0.03)
            self.assertIsNotNone(last_distance.speed)

        self.assertGreaterEqual(count, 9)
        self.assertEqual(rangefinder.distance, last_distance.value)
        self.assertAlmostEqual(rangefinder.speed, 0, delta=0.5)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    unittest.main()
