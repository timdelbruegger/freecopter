from util.definitions import SENSOR_ERROR_MAX, PRESSURE_SENSOR_HEIGHT_ERROR, START_HEIGHT_ABOVE_SEA



# Represents a reading of a pressure sensor with integrated temperature sensor.
# The individual values might be None, if the corresponding sensor does not have a valid reading at the moment.
class PressureTemperatureState:
    def __init__(self, pressureValid, pressure, temperatureValid, temperature):

        self.pressure = None
        self.height_above_sea = START_HEIGHT_ABOVE_SEA
        self.height_above_sea_error = SENSOR_ERROR_MAX
        self.temperature = None

        if pressureValid:
            self.pressure = pressure
            self.height_above_sea = computeHeight(pressure)

            self.height_above_sea_error = PRESSURE_SENSOR_HEIGHT_ERROR

        if temperatureValid:
            self.temperature = temperature


#  computeHeight() - the conversion uses the formula:
#
#  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
#
#  where:
#  h  = height above sea level
#  T0 = standard temperature at sea level = 288.15
#  L0 = standard temperatur elapse rate = -0.0065
#  p  = measured pressure
#  P0 = static pressure = 1013.25
#  g0 = gravitational acceleration = 9.80665
#  M  = mloecular mass of earth's air = 0.0289644
#  R* = universal gas constant = 8.31432
#
#  Given the constants, this works out to:
#
#  h = 44330.8 * (1 - (p / P0)**0.190263)
def computeHeight(pressure):
    return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263));