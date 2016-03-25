from smbus import SMBus
from datetime import datetime, timedelta
from util.definitions import SRF02_MIN_TIME_BETWEEN_BURSTS, SRF02_I2C_ADDRESS, SRF02_MIN_TIME_BETWEEN_BURST_READ, SRF02_MAX_WAIT_TIME
import logging

log = logging.getLogger("SRF02")


# Takes care of accessing the SRF02 ultrasonic range finder via I2C interface.
# The speciality with this sensor is that it does the timing itself. This means that we do not need a second thread
# for this and can call the update function in the normal sensor loop.
#
# Fields:
# - distance: current distance measurement in meters
# - mindistance: current estimation of the minimum distance that can be measured by the sensor
#
# See the datasheet: http://www.robot-electronics.co.uk/htm/srf02techI2C.htm
class SRF02:

    def __init__(self):

        self._i2c = SMBus(1)
        self._i2c_address = SRF02_I2C_ADDRESS
        self._waiting_for_echo = False

        yesterday = datetime.now() - timedelta(1)
        self._time_last_burst = yesterday

        # The last distance measurement
        self.distance = None  # meters

        # On power up, the detection threshold is set to 28cm (11")
        self.mindistance = 0.28  # meters

        # This is mostly for debugging and testing
        self.num_bursts_sent = 0

        # check that the sensor is present - read the version
        self.version = self._read_version()
        log.info("SRF-02 Ultrasonic Sensor initialized. Version: %d" % self.version)

        # we want to check how often we get exceptions
        self.error_counter = 0

    # Should be called in some sensor loop, maybe at 100Hz. Is fast.
    # Will trigger an ultrasonic burst or check if we received an echo.
    # I we have a measurement, it is returned in meters.
    def update(self):

        distance = None

        now = datetime.now()
        time_since_last_burst = (now - self._time_last_burst).total_seconds()

#        log.debug("time since last burst: {}".format(time_since_last_burst))

        if self._waiting_for_echo:
            # make sure we wait at least some amount of time before we read
            if time_since_last_burst > SRF02_MIN_TIME_BETWEEN_BURST_READ:
                # check if we have an echo
                distance = self._read_echo()

            # Fallback if we don't get an echo, just stop waiting
            # from the data sheet:
            # The SRF02 will always be ready 70mS after initiating the ranging.
            if distance is None and time_since_last_burst > SRF02_MAX_WAIT_TIME:
                log.warn("Fallback! Waited longer than 70ms!")
                self._waiting_for_echo = False

        if (not self._waiting_for_echo) and time_since_last_burst > SRF02_MIN_TIME_BETWEEN_BURSTS:
            self._send_burst()

        return distance

    def _send_burst(self):
        self._i2c.write_byte_data(self._i2c_address, 0, 0x51)
        self._waiting_for_echo = True
        self._time_last_burst = datetime.now()
        self.num_bursts_sent += 1
        log.debug("Burst sent.")

    def _read_echo(self):
        # it must be possible to read all of these data in 1 i2c transaction
        # buf[0] software version. If this is 255, then the ping has not yet returned
        # buf[1] unused
        # buf[2] high byte range
        # buf[3] low byte range
        # buf[4] high byte minimum auto tuned range
        # buf[5] low byte minimum auto tuned range

        # We use the version information to detect if the result is there yet.
        # 255 is a dummy version for the case that no echo has been received yet. For me, the real version is "6".
        if self._read_version() == 255:
            return None

        self.distance = self._i2c.read_word_data(self._i2c_address, 2) / 255 / 100
        self.mindistance = self._i2c.read_word_data(self._i2c_address, 4) / 255 / 100

        # A value of 0 indicates that no objects were detected. We prefer None to represent this.
        if self.distance == 0:
            self.distance = None

        self._waiting_for_echo = False
        log.debug("echo received! distance is: {}".format(self.distance))

        return self.distance

    def _read_version(self):
        try:
            return self._i2c.read_byte_data(self._i2c_address, 0)

            # 255 means that the unit is still measuring the distance
        except IOError:
            log.error("Recovering from IOError")
            self.error_counter += 1
            return 255
