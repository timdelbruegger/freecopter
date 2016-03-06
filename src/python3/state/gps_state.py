from util.definitions import SENSOR_ERROR_MAX
from math import isnan, isfinite


# Represents one reading from a GPS device
# See GPSd description: http://www.catb.org/gpsd/gpsd_json.html
class GPSState:
    def __init__(self, data):

        # for debugging purposes
        self.rawfix = data.fix

        self.utc = data.utc
        self.time = data.fix.time

        self.num_satellites = len(data.satellites)

        self.latitude = data.fix.latitude
        self.longitude = data.fix.longitude

        # Longitude error estimate in meters, 95% confidence.
        # Present if mode is 2 or 3 and DOPs can be calculated from the satellite view.
        self.longitude_error = read_error(data.fix, 'epx')

        # Latitude error estimate in meters, 95% confidence.
        # Present if mode is 2 or 3 and DOPs can be calculated from the satellite view.
        self.latitude_error = read_error(data.fix, 'epy')

        # Climb/sink error estimate in meters/sec, 95% confidence.

        # altitude in meters
        self.altitude = data.fix.altitude
        if isnan(self.altitude) or not isfinite(self.altitude):
            # just some value so that the calculation can continue
            # this will have next to no influence because the expected error is so high in this situation
            self.altitude = 50

        # accuracy depends on number of satellites
        # Estimated vertical error in meters, 95% confidence.
        # Present if mode is 3 and DOPs can be calculated from the satellite view.
        self.altitude_error = read_error(data.fix, 'epv')

        # Speed over ground, meters per second.
        self.speed = data.fix.speed

        # Speed error estimate in meters/sec, 95% confidence.
        self.speed_error = read_error(data.fix, 'eps')

        # Climb (positive) or sink (negative) rate, meters per second.
        self.climb = data.fix.climb

        # Climb/sink error estimate in meters/sec, 95% confidence.
        self.climb_error = read_error(data.fix, 'epc')

        # Course over ground, degrees from true north.
        self.track = data.fix.track

        # Direction error estimate in degrees, 95% confidence.
        self.track_error = read_error(data.fix, 'epd')

        # NMEA mode: %d, 0=no mode value yet seen, 1=no fix, 2=2D, 3=3D.
        self.rawmode = data.fix.mode

        # flags for different modes
        self.has_no_fix = data.fix.mode <= 1
        self.has_2d_fix = data.fix.mode >= 2
        self.has_3d_fix = data.fix.mode == 3


# If present, returns the attribute of the given name in data.
# If the attribute is not present, ERROR_MAX is returned.
def read_error(data, attribute_name):
    if hasattr(data, attribute_name):
        attr = getattr(data, attribute_name)
        if not isnan(attr) and isfinite(attr):
            return attr

    return SENSOR_ERROR_MAX
