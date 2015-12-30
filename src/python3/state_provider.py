import sys
import RTIMU
import logging
from datetime import datetime

from sensors.range_finder_process import UltrasonicRangeFinderProcess
from state_change_planning_3d import State3d
from state_change_planning import State
import os.path

# contains settings and calibration of accelerometer/gyroscope/magnetometer
RT_IMU_LIB_SETTINGS_FILE = "RTIMULib"

# In a 3d vector with indizes (0,1,2), this index is for the "Up" direction
UP_AXIS_INDEX = 1

# define GPIO pins for downward ultrasonic range finder
RANGE_FINDER_DOWN_Trigger = 17
RANGE_FINDER_DOWN_Echo = 18


# Accumulates and fuses data from all sensors for a complete sense of the current state.
# Mostly delegates to RTIMULib for a first implementation
class StateProvider:
    def __init__(self):
        self.log = logging.getLogger("StateProvider")
        self.__listeners = []

        self.log.info("Using settings file " + RT_IMU_LIB_SETTINGS_FILE + ".ini")
        if not os.path.exists(RT_IMU_LIB_SETTINGS_FILE + ".ini"):
            self.log.info("Settings file does not exist, will be created")

        self.settings = RTIMU.Settings(RT_IMU_LIB_SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(self.settings)
        self.pressure = RTIMU.RTPressure(self.settings)

        self.log.info("IMU Name: " + self.imu.IMUName())
        self.log.info("Pressure Name: " + self.pressure.pressureName())

        if not self.imu.IMUInit():
            self.log.critical("IMU Init Failed. Exiting...")
            sys.exit(1)
        else:
            self.log.info("IMU Init Succeeded");

        # this is a good time to set any fusion parameters

        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        if not self.pressure.pressureInit():
            self.log.critical("Pressure sensor Init Failed. Exiting...")
            sys.exit(1)
        else:
            self.log.info("Pressure sensor Init Succeeded")

        # we should try to ask for new data every x seconds
        # IMUGetPollInterval returns milliseconds, we translate to seconds
        # print("Recommended Poll Interval: %dms\n" % self.imu.IMUGetPollInterval())
        self.pollInterval = self.imu.IMUGetPollInterval() / 1000.0
        self.log.info("Used Poll Interval: %fs\n" % self.pollInterval)

        self.log.info("Initiating ultrasonic range finder")
        self.downDistanceSensor = UltrasonicRangeFinderProcess(RANGE_FINDER_DOWN_Trigger, RANGE_FINDER_DOWN_Echo)
        self.downDistanceSensor.start()

        self.log.info("Done! We can start...")

        self.firstReading = True
        self.startHeightAboveSea = None
        self.lastUpdate = datetime.now()


    def registerListener(self, listener):
        self.log.debug("Appending listener: " + listener.name)
        self.__listeners.append(listener)

    # Combines IMU and Pressure readings into a full quadrocopter state
    def _read_state(self, imu_data, pressure_data, elevation_state):
        # here we compile the data
        newstate = QuadrocopterState()

        # collect fused data
        newstate.raw = imu_data

        # add the pressure data
        (air_pressure_valid, air_pressure, temperature_valid, temperature) = pressure_data

        self.log.debug("---------------  pressureRead  ---------------------")
        self.log.debug("air pressure valid: " + str(air_pressure_valid))
        self.log.debug("air pressure: " + str(air_pressure))
        self.log.debug("temperature valid: " + str(temperature_valid))
        self.log.debug("temperature: " + str(temperature))
        self.log.debug("------------------------------------")

        # copy so that everything is in one place
        newstate.raw["pressureValid"] = air_pressure_valid
        newstate.raw["pressure"] = air_pressure
        newstate.raw["temperatureValid"] = temperature_valid
        newstate.raw["temperature"] = temperature

        self.log.debug("---------------  getIMUData  ---------------------")
        self.log.debug(newstate.raw)

        if temperature_valid:
            newstate.temperature = temperature
        else:
            self.log.warn("Temperature reading is not valid!")

        if newstate.raw["accelValid"]:
            newstate.body_frame_acceleration = newstate.raw["accel"]
        else:
            self.log.error("Accelerometer reading is not valid!")

        # pressure should always be valid, but the code should not crash if we get a False here.
        if air_pressure_valid:
            newstate.airPressure = air_pressure
            newstate.heightAboveSea = compute_height(newstate.airPressure)

            # special operation for first reading
            if self.firstReading:
                # TODO: we should use the average pressure over all measurements before liftoff
                self.startHeightAboveSea = newstate.heightAboveSea
                self.firstReading = False

            # calculate height above starting point
            # TODO: At the moment we have no simple way of getting the vertical speed
            #       Maybe this gets better with ultrasonic sensors
            newstate.elevation = State.by_value_and_speed(newstate.heightAboveSea - self.startHeightAboveSea, 0)
        else:
            self.log.warn("pressure reading is not valid!")

        (newstate.rotation.x.value, newstate.rotation.y.value, newstate.rotation.z.value) = newstate.raw["fusionPose"]
        (newstate.rotation.x.speed, newstate.rotation.y.speed, newstate.rotation.z.speed) = newstate.raw["gyro"]

        # TODO: For now, we don't take the orientation into account here
        # Of course this is not correct if the quadrocopter is not even in the air.
        newstate.elevation = elevation_state

        return newstate

    def update(self):
        finished = False
        while not finished:
            if self.imu.IMURead():

                finished = True

                newstate = self._read_state(self.imu.getIMUData(), self.pressure.pressureRead(), self.downDistanceSensor.read_distance())

                # new state is ready, we can print it
                self.log.debug(newstate)

                # give new state to listeners
                current_time = datetime.now()
                time_since_last_update = (current_time - self.lastUpdate).total_seconds()
                for listener in self.__listeners:
                    listener.new_sensor_reading(time_since_last_update, newstate)

                # save time for next iteration
                self.lastUpdate = current_time

        self.downDistanceSensor.stop()


# Defines the current best estimate of the complete state of the quadrocopter.
# This includes the height above the starting position and the orientation in the air and will
# include other measures like the position relative to the starting position, absolute
# position on earth (mainly via GPS), distance to ground (ultrasonic sensors, world perception).
# This data about the current state of the quadrocopter is further extended by environmental measures
# like air pressure, temperature, maybe wind speed and direction.
class QuadrocopterState:
    def __init__(self):
        self.elevation = State()
        self.rotation = State3d()
        self.body_frame_acceleration = None

        # temperature in Celsius
        self.temperature = None
        self.airPressure = None
        self.heightAboveSea = None

        self.raw = None

        # TODO: include linear speeds
        # TODO: include earth frame speeds/accelerations (oriented north and y axis away from earth)
        # TODO: include GPS position via external service
        pass

    def __str__(self):
        return "QuadrocopterState = {" + str(self.__dict__) + "}"


# computeHeight() - the conversion uses the formula:
#
#  h = (T0 / L0) * ((p / P0)**(-(R* * L0) / (g0 * M)) - 1)
#
#  where:
#  h  = height above sea level
#  T0 = standard temperature at sea level = 288.15
#  L0 = standard temperature elapse rate = -0.0065
#  p  = measured pressure
#  P0 = static pressure = 1013.25
#  g0 = gravitational acceleration = 9.80665
#  M  = molecular mass of earth's air = 0.0289644
#  R* = universal gas constant = 8.31432
#
#  Given the constants, this works out to:
#
#  h = 44330.8 * (1 - (p / P0)**0.190263)

def compute_height(pressure):
    return 44330.8 * (1 - pow(pressure / 1013.25, 0.190263));
