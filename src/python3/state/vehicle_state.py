from state.pressure_temp_state import PressureTemperatureState
from state.attitude_state import AttitudeState
from state.height_state import HeightState
from state.gps_state import GPSState


# Represents all the knowledge about our quadrocopter that we have at one point in time
class VehicleState:
    def __init__(self, attitude, height, gps, pressure_temp):

        assert(isinstance(attitude, AttitudeState))
        assert(isinstance(height, HeightState))
        assert(isinstance(gps, GPSState))
        assert(isinstance(pressure_temp, PressureTemperatureState))

        # maybe repackage this information?
        self.attitude = attitude
        self.height = height

        # this is a little low level, but the information should not be lost...
        self.gps = gps

        self.air_pressure = pressure_temp.pressure
        self.temperature = pressure_temp.temperature

    def fields(self):
        return {
            "height_above_ground": self.height.height_above_ground,
            "temperature": self.temperature}