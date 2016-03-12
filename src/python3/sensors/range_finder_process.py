from multiprocessing import Process, Queue, Value
from sensors.range_finder import UltrasonicRangeFinder
from util.definitions import SENSOR_ERROR_MAX, ULTRASONIC_SENSOR_ERROR
from quaternion import Quaternion

from util.definitions import FORWARD_AXIS, UP_AXIS

import math
import numpy

time_between_measurements = 0.001


def _range_finder_process(gpio_trigger, gpio_echo, queue, stop_flag):

    rangefinder = UltrasonicRangeFinder(gpio_trigger, gpio_echo)
    rangefinder.measure(queue, stop_flag, time_between_measurements)


# Because the ultrasonic range finder uses busy waiting on an echo, we don't want
# to block the execution thread and rather use a second process. This class wraps
# the process so that it can be used as if it were the range finder directly.
class UltrasonicRangeFinderProcess:

    def __init__(self, gpio_trigger, gpio_echo):
        self.distance_queue = Queue()
        self.stop_flag = Value('i', False)
        self.process = Process(
                target=_range_finder_process,
                args=(gpio_trigger, gpio_echo, self.distance_queue, self.stop_flag))

        self.distance = None

    def start(self):
        self.process.start()

    def stop(self):
        self.stop_flag.value = True
        self.process.join()

    # The orientation quaternion is used to estimate the accuracy: the more even we stand towards ground,
    # the more exact is our measurement of the height above ground
    def read_distance(self, orientation_quaternion):

        # get the newest reading from the queue without blocking
        while not self.distance_queue.empty():
            self.distance = self.distance_queue.get(False)

        if self.distance is None:
            return 0.0, SENSOR_ERROR_MAX
        else:
            # Second value is expected error
            return self.distance, _estimated_error_above_ground(orientation_quaternion)


def _estimated_error_above_ground(orientation_quaternion):
    assert(isinstance(orientation_quaternion, Quaternion))

    angle_to_up = _calc_angle_to_up_axis(orientation_quaternion)

    # from 0 degree to 45 degrees we interpolate
    ultrasonic_clamp_rate = (SENSOR_ERROR_MAX-ULTRASONIC_SENSOR_ERROR) / (math.pi * 0.25)
    expected_error = ULTRASONIC_SENSOR_ERROR + angle_to_up * ultrasonic_clamp_rate

    # if the expected error is more than 2 meters, just ignore our value
    return min(expected_error, SENSOR_ERROR_MAX)


def _calc_angle_to_up_axis(orientation_quaternion):
    # The quaternions in RTIMULib are thought to start at (0,0,1)
    # turn Z by orientation quaternion
    direction = orientation_quaternion.rotate(FORWARD_AXIS)
    # this is the angle between the ultrasonic beam and the real direction to ground
    # Now we have a right triangle, distance = length of hypothenuse
    # angle = arccos(dot(ultrasonic_beam, UP))
    # cos(angle) = height / distance
    return math.arccos(numpy.dot(direction, UP_AXIS))
