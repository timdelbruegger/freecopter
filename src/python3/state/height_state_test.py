import unittest
from state.height_state import HeightState
from numpy import array

imu_reading = {'pressureValid': False,
                 'accelValid': True,
                 'temperature': 0.0,
                 'pressure': 0.0,
                 'fusionQPoseValid': True,
                 'timestamp': 1456588586361310,
                 'compassValid': True,
                 'compass': (1.751953363418579, -5.758125305175781, 11.735391616821289),
                 'accel': (0.049072265625,
                           0.008544921875, 1.035888671875),
                 'humidity': 0.0,
                 'gyroValid': True,
                 'gyro': (-0.05853237956762314, 0.005321125499904156,
                          -0.011706476099789143),
                 'temperatureValid': False,
                 'humidityValid': False,
                 'fusionQPose': (0.7744086980819702, 0.018158545717597008, -0.01572602242231369, 0.6322295069694519),
                 'fusionPoseValid': True,
                 'fusionPose': (0.008248693309724331, -0.04733514413237572, 1.3691307306289673)}


vertical_speed = 13
height_above_ground = 200
ground_height_barometer = 10
ground_height_gps = -1


# We will just try to create AttitudeStates from the sample IMU reading
class TestHegState(unittest.TestCase):

    def test_set_and_vector_transform(self):

        state = HeightState(height_above_ground, vertical_speed, ground_height_barometer, ground_height_gps)

        self.assertEqual(state.height_above_ground, height_above_ground)
        self.assertEqual(state.vertical_speed, vertical_speed)
        self.assertEqual(state.ground_height_barometer, ground_height_barometer)
        self.assertEqual(state.ground_height_gps, ground_height_gps)

        vector = state.as_vector()
        self.assertEqual(vector[0], state.height_above_ground)
        self.assertEqual(vector[1], state.vertical_speed)
        self.assertEqual(vector[2], state.ground_height_barometer)
        self.assertEqual(vector[3], state.ground_height_gps)

    def test_set_from_vector1(self):
        vector = array([height_above_ground, vertical_speed, ground_height_barometer, ground_height_gps])
        state = HeightState.fromVector(vector)

        self.assertEqual(state.height_above_ground, height_above_ground)
        self.assertEqual(state.vertical_speed, vertical_speed)
        self.assertEqual(state.ground_height_barometer, ground_height_barometer)
        self.assertEqual(state.ground_height_gps, ground_height_gps)

    def test_set_from_vector2(self):
        vector = array([[height_above_ground], [vertical_speed], [ground_height_barometer], [ground_height_gps]])
        state = HeightState.fromVector(vector)

        self.assertEqual(state.height_above_ground, height_above_ground)
        self.assertEqual(state.vertical_speed, vertical_speed)
        self.assertEqual(state.ground_height_barometer, ground_height_barometer)
        self.assertEqual(state.ground_height_gps, ground_height_gps)


if __name__ == '__main__':
    unittest.main()
