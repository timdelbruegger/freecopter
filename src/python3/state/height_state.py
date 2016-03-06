from numpy import array, ndarray
import json
from math import isnan


# This is the state of the Altitude Sensor Fusion (based on a Kalman Filter)
# See https://timdelbruegger.wordpress.com/2016/01/05/altitude-sensor-fusion/
class HeightState:

    def __init__(self, height_above_ground, vertical_speed, ground_height_barometer, ground_height_gps):

        assert(not isnan(height_above_ground))
        assert(not isnan(vertical_speed))
        assert(not isnan(ground_height_barometer))
        assert(not isnan(ground_height_gps))

        self.height_above_ground = height_above_ground
        self.vertical_speed = vertical_speed
        self.ground_height_barometer = ground_height_barometer
        self.ground_height_gps = ground_height_gps

    def as_vector(self):
        return array([[self.height_above_ground],
                      [self.vertical_speed],
                      [self.ground_height_barometer],
                      [self.ground_height_gps]])

    @classmethod
    def fromVector(cls, vec):
        assert len(vec) == 4

        if isinstance(vec, ndarray):
            assert vec.shape == (4,) or vec.shape == (4, 1)

        return HeightState(vec[0], vec[1], vec[2], vec[3])