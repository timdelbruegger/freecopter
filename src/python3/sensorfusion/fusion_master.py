from state.logging_state_provider import LoggingStateProviderWithListeners
from sensorfusion.height_provider import HeightProvider
from sensorfusion.attitude_provider import AttitudeProvider, AttitudeState

class SensorFusionMaster(LoggingStateProviderWithListeners):
    """
    This class is responsible for taking the single sensor fusion results and integrating them into a VehicleState.
    - AttitudeProvider (gyro, accelerometer, magnetometer) -> AttitudeState (orientation in air)
    - HeightProvider (gps, ultrasonic sensor, barometer, attitude) -> HeightState (height above ground, vertical speed)
    """
    def __init__(self):
        self.attitudeProvider = AttitudeProvider()
        self.heightProvider = HeightProvider()

    def update(self):
        # this will trigger the heightProvider via the listener
        attitude = self.attitudeProvider.update()
        vehicle_state = self.heightProvider.update(attitude)

        return vehicle_state
