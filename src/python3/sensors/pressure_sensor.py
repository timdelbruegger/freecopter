import RTIMU

import logging
from state.pressure_temp_state import PressureTemperatureState


# Represents a sensor that measures air pressure and temperature
class PressureTemperatureSensor:
    def __init__(self, rt_imu_settings):
        self.log = logging.Logger("PressureSensor")
        self.pressure = RTIMU.RTPressure(rt_imu_settings)
        self.log.info("Pressure Sensor: " + self.pressure.pressureName())

        if not self.pressure.pressureInit():
            self.log.error("Pressure sensor Init Failed")
        else:
            self.log.info("Pressure sensor Init Succeeded")

    def read(self):

        (pressureValid, pressure, temperatureValid, temperature) = self.pressure.pressureRead()
        return PressureTemperatureState(pressureValid=pressureValid,
                                        pressure=pressure,
                                        temperatureValid=temperatureValid,
                                        temperature=temperature)