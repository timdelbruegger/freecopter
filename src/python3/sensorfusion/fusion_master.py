from state.logging_state_provider import LoggingStateProviderWithListeners
from sensorfusion.height_provider import HeightProvider
from sensorfusion.attitude_provider import AttitudeProvider, AttitudeState


# This class is responsible for taking the single sensor fusion results and integrating them into a CopterState
class SensorFusionMaster(LoggingStateProviderWithListeners):
    def __init__(self):
        self.attitudeProvider = AttitudeProvider()
        self.heightProvider = HeightProvider()
        self.attitudeProvider.registerListener(self.heightProvider)

    def readState(self):
        # this will trigger the heightProvider via the listener
        attitude = self.attitudeProvider.readSync()
        vehicle_state = self.heightProvider.update(attitude)

        return vehicle_state