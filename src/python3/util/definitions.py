from numpy import array
import RTIMU

# define GPIO pins
GPIO_ULTRASONIC_TRIGGER = 17
GPIO_ULTRASONIC_ECHO = 18

# contains settings and calibration of accelerometer/gyroscope/magnetometer
RT_IMU_LIB_SETTINGS_FILE = "RTIMULib"

# The settings object
RT_IMU_SETTINGS = RTIMU.Settings(RT_IMU_LIB_SETTINGS_FILE)

# In a 3d vector with indizes (0,1,2), this index is for the "Up" direction
UP_AXIS_INDEX = 1
UP_AXIS = array([0, 1, 0])
FORWARD_AXIS = array([0, 0, 1])

# Maximum expected error that might possibly happen.
# This is used if no data is available, for example if we don't have enough satellites for a GPS fix.
SENSOR_ERROR_MAX = 10000000

# expected error in meters
# TODO: make sure that this is really the expected error for this kind of sensor
PRESSURE_SENSOR_HEIGHT_ERROR = 40

# expected error in meters
# TODO: Make sure the expected error is correct for the used range finder
ULTRASONIC_SENSOR_ERROR = 0.03

# we initialize the barometer state with this height above sea level.
# The value does not really matter because in the beginning the expected error is really big.
START_HEIGHT_ABOVE_SEA = 0