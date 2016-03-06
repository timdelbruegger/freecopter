import logging

from numpy import *

from sensorfusion.kalman import KalmanFilter
from state.logging_state_provider import LoggingStateProviderWithListeners
from state.height_state import HeightState
from sensors.pressure_sensor import PressureTemperatureSensor
from sensors.range_finder_process import UltrasonicRangeFinderProcess
from sensors.gps_polling_thread import GpsPollingThread
from util.timer import Timer
from state.vehicle_state import VehicleState
from util.definitions import *
from pyquaternion import Quaternion


# F: state transition matrix
def F(delta_time):
    # diagonal = 1, so we copy the old state as a basis for the next
    # the vertical speed influences the height linearly in time
    return array([
        [1, delta_time, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])


# B: influence of vertical acceleration on the state vector
def B(delta_time):
    # vertical acceleration influences the altitude quadratically
    # vertical acceleration influences the vertical speed linearly
    # vertical acceleration does not influence GPS / barometer ground offsets
    return array([[0.5*square(delta_time)], [delta_time], [0], [0]])


# Reads Barometer + ultrasonic sensor + GPS and fuses them with accelerometer to gather good knowledge of the
# height above ground. See https://timdelbruegger.wordpress.com/2016/01/05/altitude-sensor-fusion/
class HeightProvider(LoggingStateProviderWithListeners):

    def __init__(self, rt_imu_settings, ultrasonic_gpio_trigger, ultrasonic_gpio_echo, gps_enabled=False):
        super().__init__("HeightProvider")

        self.log.debug("setup sensors...")
        self.barometer = PressureTemperatureSensor(RT_IMU_SETTINGS)
        self.ultrasonic = UltrasonicRangeFinderProcess(ultrasonic_gpio_trigger, ultrasonic_gpio_echo)

        self.gps = GpsPollingThread(gps_enabled)
        if gps_enabled:
            self.log.info("Start reading from GPS...")
            self.gps.start()
            self.log.info("Reading from GPS started.")
        else:
            self.log.info("GPS is not used.")

        self.log.debug("Setting up Kalman Filter...")

        # initial state vector X
        # We assume to be at the ground with speed zero
        x = HeightState(0, 0, 100, 100).as_vector()
        self.log.debug("x: ", x)

        # P: covariance matrix at k-1
        # We set the initial probability distribution to be quite confident in the first two state dimensions and very
        # unsure in the GPS / Barometer ground offsets:
        P = diag([0.1, 0.1, 10000, 10000])
        self.log.debug("P: ", P)

        # Q: predict noise covariance matrix
        # The state prediction only affects height above ground and vertical speed. Since the influence of the noisy
        # accelerometer reading is stronger on the vertical speed, the noise is stronger there.
        Q = diag([0.3, 0.5, 0, 0])
        self.log.debug("Q: ", Q)

        # H: measurement prediction matrix
        H = array([[1, 0, 1, 0],  # barometer height measurement
                   [1, 0, 0, 0],  # ultrasonic height measurement
                   [1, 0, 0, 1]])  # gps height measurement
        self.log.debug("H: ", H)

        self.kf = KalmanFilter(x=x, P=P, A=F, Q=Q, B=B, H=H)

        self.timer = Timer()

        self.log.debug("HeightProvider is ready.")

    # perform an update after the given amount of time
    # dt: seconds since last update
    def update(self, attitude_state):

        dt = self.timer.readAndReset()

        # read data from sensors
        # It does not matter if the sensors are not ready, because then the variance will be very big
        # and other sensors and the state transition will take over.
        baro_reading = self.barometer.read()
        (dist_ultrasonic, ultrasonic_error) = self.ultrasonic.read_distance(attitude_state.orientation)
        gps_reading = self.gps.read()

        # TODO: correct ultrasonic measurement by angle to ground normal
        # maybe we don't need this as the ultrasonic wave has some width? For now,
        # we only adjust the accuracy based on the angle
        height_ultrasonic = dist_ultrasonic

        # calculate vertical acceleration based on accelerometer data
        vertical_acceleration = calculate_up_acceleration(attitude_state.orientation, attitude_state.acceleration)

        # control input
        u = array([vertical_acceleration])
        self.log.debug("U: ", u)

        # Kalman Step 1: Predict with input
        self.log.debug("Predict with Input")
        (x, P) = self.kf.predictWithInput(u, dt)
        self.log.debug("x: ", x)
        self.log.debug("P: ", P)


        # Y: measurement vector
        # TODO: we could also add the GPS climb speed measurement
        Y = array([[baro_reading.height_above_sea], [height_ultrasonic], [gps_reading.altitude]])

        # R: measurement noise covariance matrix
        R = diag([baro_reading.height_above_sea_error, ultrasonic_error, gps_reading.altitude_error])

        # Kalman Step 2: Update with measurements
        self.log.debug("Y: ", Y)
        self.log.debug("R: ", R)
        (x, P) = self.kf.updateWithMeasurement(Y, R)

        self.log.debug("X: ", x)

        newstate = VehicleState(attitude_state, HeightState.fromVector(x), gps_reading, baro_reading)
        self.notify_listeners(newstate)
        return newstate


# Calculates the distance above ground from an ultrasonic measurement facing down.
# The ground is assumed to be planar with normal (0,1,0).
# The ultrasonic sensor beam is assumed to be a ray.
# distance is assumed to be meters
# TODO: Check how narrow the sensor beam really is
def correct_ultrasonic_angle(orientation, distance):

    assert(isinstance(orientation, Quaternion))
    assert(isinstance(distance, float))

    # The quaternions in RTIMULib are thought to start at (0,0,1)
    # turn Z by orientation quaternion
    direction = orientation.rotate(FORWARD_AXIS)

    # this is the angle between the ultrasonic beam and the real direction to ground
    # Now we have a right triangle, distance = length of hypothenuse
    # angle = arccos(dot(ultrasonic_beam, UP))
    # cos(angle) = height / distance
    height = dot(direction, UP_AXIS) * distance

    return height


# projects the body frame acceleration vector onto the world UP vector
def calculate_up_acceleration(orientation, acceleration_body_frame):
    acceleration_world_frame = body_to_world_frame(orientation, acceleration_body_frame)

    # take Y coordinate
    return acceleration_world_frame[1]


# Transforms a body frame direction vector to world centric coordinates based on a given orientation quaternion
def body_to_world_frame(orientation, vector):

    # revert the body orientation to get the world coordinates
    return orientation.inverse().rotate(vector)