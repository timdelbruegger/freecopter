from util.definitions import *
import math
import numpy
from pyquaternion import Quaternion


class RangeFinderHeightAboveGroundAdapter:
    """ Decorator for a range finder so that an ultrasonic range finder can be used to estimate the height above ground.
    Assumes that the ultrasonic range finder is pointing down.
    We make sure that:
     - the reported distance is not None at all times
     - the expected error is not None at all times
     - the expected error is adjusted to the attitude so that only small angles contribute to the height above ground.
    """

    def __init__(self, range_finder):
        self.range_finder = range_finder

    def update(self, attitude):
        # print(type(attitude).__name__)
        # print(attitude)
        assert(isinstance(attitude, Quaternion))

        # update range finder
        distance, error_plain = self.range_finder.update()

        # if we have no measurement, the error is at maximum, no need to adjust
        if distance is None:
            if self.range_finder.distance is not None:
                # use last distance reading
                # it does not really matter because the expected error is maximal
                return self.range_finder.distance, error_plain
            else:
                # just use some default
                # it does not really matter because the expected error is maximal
                return 0.0, error_plain

        # we have a measurement, so we have to correct the expected error
        error_corrected = error_plain * calc_error_multiplier(attitude)

        # make sure that the error is not higher than max
        error_corrected = min(error_corrected, SENSOR_ERROR_MAX)

        # return measured distance with the corrected error expectation
        return distance, error_corrected


def calc_error_multiplier(attitude):
    angle_to_up = _calc_angle_to_up_axis(attitude) / math.pi
    # print("angle_to_up = {}".format(angle_to_up))

    # smaller than 45 degrees is no problem, the range finder waves have a rather wide angle
    if angle_to_up < 0.25:
        return 1.0 + angle_to_up**2

    # at least 45 degrees: difficult to say, is it really the ground or some side object?
    return SENSOR_ERROR_MAX


def _calc_angle_to_up_axis(orientation_quaternion):
    # The quaternions in RTIMULib are thought to start at (FORWARD_AXIS)
    # turn Z by orientation quaternion
    # forward_direction = orientation_quaternion.rotate(FORWARD_AXIS)
    # right_direction = orientation_quaternion.rotate(RIGHT_AXIS)
    up_direction = orientation_quaternion.rotate(UP_AXIS)

    # print("forward_direction = {}".format(forward_direction))
    # print("right_direction = {}".format(right_direction))
    # print("up_direction = {}".format(up_direction))

    # this is the angle between the ultrasonic beam and the real direction to ground
    # Now we have a right triangle, distance = length of hypothenuse
    # angle = arccos(dot(ultrasonic_beam, UP))
    # cos(angle) = height / distance
    return math.acos(numpy.dot(up_direction, UP_AXIS))
