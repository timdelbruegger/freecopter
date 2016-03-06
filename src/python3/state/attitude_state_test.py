import unittest
from state.attitude_state import AttitudeState

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


# We will just try to create AttitudeStates from the sample IMU reading
class TestAttitudeState(unittest.TestCase):

    def test_sample_1(self):
        state = AttitudeState(imu_reading)

        # raw state is just a copy
        self.assertEqual(state.raw, imu_reading)

        self.assertEqual(state.rotation.x.value, imu_reading["fusionPose"][0])
        self.assertEqual(state.rotation.y.value, imu_reading["fusionPose"][1])
        self.assertEqual(state.rotation.z.value, imu_reading["fusionPose"][2])

        self.assertEqual(state.rotation.x.speed, imu_reading["gyro"][0])
        self.assertEqual(state.rotation.y.speed, imu_reading["gyro"][1])
        self.assertEqual(state.rotation.z.speed, imu_reading["gyro"][2])

        self.assertEqual(state.orientation, imu_reading["fusionQPose"])

        self.assertEqual(state.acceleration, imu_reading["accel"])


if __name__ == '__main__':
    unittest.main()
