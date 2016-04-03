# uses RTIMULib in order to get a fast attitude estimation


import logging
import os.path
import sys
from datetime import datetime

import RTIMU

from state.attitude_state import AttitudeState
from state.logging_state_provider import LoggingStateProviderWithListeners

from util.definitions import *



class AttitudeProvider (LoggingStateProviderWithListeners):
    """
    Accumulates data from gyroscope, accelerometer, magnetometer into one AttitudeState that describes the orientation
    of the vehicle in the air. Uses the RTIMULib Attitude Sensor Fusion.
    """
    def __init__(self):

        super().__init__("AttitudeProvider")

        self.log.info("Using settings file " + RT_IMU_LIB_SETTINGS_FILE + ".ini")
        if not os.path.exists(RT_IMU_LIB_SETTINGS_FILE + ".ini"):
            self.log.info("Settings file does not exist, will be created")

        self.settings = RTIMU.Settings(RT_IMU_LIB_SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(self.settings)

        self.log.info("IMU Name: " + self.imu.IMUName())

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

        # we should try to ask for new data every x seconds
        # IMUGetPollInterval returns milliseconds, we translate to seconds
        # print("Recommended Poll Interval: %dms\n" % self.imu.IMUGetPollInterval())
        self.pollInterval = self.imu.IMUGetPollInterval() / 1000.0
        self.log.info("Used Poll Interval: %fs\n" % self.pollInterval)

        self.log.info("Done! We can start...")

        self.state = None

    def _read_state(self, imu_data):
        """
        Combines IMU and Pressure readings into a state
        :param imu_data: dictionary of data from RTIMULib
        :return: AttitudeState (describing the orientation of the vehicle)
        """

        self.log.debug("---------------  AttitudeProvider: read state  ---------------------")
        self.log.debug(imu_data)

        if not imu_data["accelValid"]:
            self.log.error("Accelerometer reading is not valid!")

        if not imu_data["gyroValid"]:
            self.log.error("Gyroscope reading is not valid!")

        return AttitudeState(imu_data)

    def update(self):
        """ Busily waits for the next IMU sensor reading (this should not take too long).
        Then notifies listeners of the new state.
        :return: The new AttitudeState (describing the orientation of the vehicle)
        """
        finished = False
        while not finished:
            if self.imu.IMURead():
                newstate = self._read_state(self.imu.getIMUData())
                self.notify_listeners(newstate)
                finished = True

                return newstate

