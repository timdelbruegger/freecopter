from state_change_planning_3d import State3d

from quaternion import Quaternion


# Represents the orientation of the vehicle at one point in time.
# Some additional information like the acceleration is also present because the sensors also give this information.
class AttitudeState:

    # construct an AttitudeState from an RTIMULib data reading
    def __init__(self, imu_data):

        # keep a copy of the raw IMU readings, helpful for debugging
        self.raw = imu_data

        # here we have roll, pitch, yaw and the according speeds of change
        self.rotation = State3d()
        (self.rotation.x.value, self.rotation.y.value, self.rotation.z.value) = imu_data["fusionPose"]
        (self.rotation.x.speed, self.rotation.y.speed, self.rotation.z.speed) = imu_data["gyro"]

        # Orientation quaternion
        # order is [w, x, y, z]
        self.orientation = Quaternion(array=imu_data["fusionQPose"])

        # TODO: where is the magnetometer?

        # acceleration in body frame
        self.acceleration = self.raw["accel"]