from numpy import array
import RTIMU

# define GPIO pins
GPIO_ULTRASONIC_TRIGGER = 17
GPIO_ULTRASONIC_ECHO = 18

# contains settings and calibration of accelerometer/gyroscope/magnetometer
RT_IMU_LIB_SETTINGS_FILE = "RTIMULib"

# The settings object
RT_IMU_SETTINGS = RTIMU.Settings(RT_IMU_LIB_SETTINGS_FILE)

# This is the RTIMULib RTIMU_XEAST_YNORTH 5
# In a 3d vector with indizes (0,1,2), this index is for the "Up" direction
UP_AXIS_INDEX = 2
UP_AXIS = array([0, 0, 1])
FORWARD_AXIS = array([1, 0, 0])
RIGHT_AXIS = array([0, 1, 0])
YAW_AXIS = UP_AXIS
PITCH_AXIS = RIGHT_AXIS
ROLL_AXIS = FORWARD_AXIS

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

# The ultrasonic range finder SR02's i2c address as reported by "i2detect -y 1"
# For me, this is 0x70 or
SRF02_I2C_ADDRESS = 0x70

# minimum wait time after a burst before we try to read
# This is not needed according to the specs. It is rather a workaround to prevent IOErrors that occur
# when we read although the measurement is not done yet.
SRF02_MIN_TIME_BETWEEN_BURST_READ = 0.065

# From the data sheet: Do not initiate a ranging faster than every 65mS to give the previous burst time to fade away.
# specs say: 65ms
SRF02_MIN_TIME_BETWEEN_BURSTS = 0.065

# specs say: 70ms
SRF02_MAX_WAIT_TIME = 0.070

# maximum distance that can be measured in meters
SRF02_MAX_RANGE = 5

# expected error (in meters) of the reported distance if the reported distance is lower or near to the current minimum range (approx. 15cm)
SRF02_SENSOR_ERROR_LOW_RANGE = 0.15

# expected error (in meters) of the reported distance if the reported distance is between the current minimal range (approx. 15cm) and SRF02_MAX_RANGE
SRF02_SENSOR_ERROR_GOOD_RANGE = 0.01

# expected error (in meters) of the reported distance if the reported distance is higher than SRF02_MAX_RANGE
SRF02_SENSOR_ERROR_HIGH_RANGE = 1
