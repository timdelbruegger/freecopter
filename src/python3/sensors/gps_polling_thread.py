import logging
import threading

import gps
from state.gps_state import GPSState


# see https://learn.adafruit.com/adafruit-ultimate-gps-on-the-raspberry-pi
class GpsPollingThread(threading.Thread):
    # If the flag enabled is False, no polling is done and empty data packages are returned.
    def __init__(self, enabled=True):
        super().__init__()

        self.log = logging.Logger("GpsPollingThread created")

        if enabled:
            # start listening to GPSD on localhost (default port 2947)
            self.gps = gps.GPS()
            self.gps.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
        else:
            self.gps = gps.GPSData()

        self.running = False

    # Starts the GPS polling thread
    def run(self):
        self.running = True
        self.log.info("Starting to read from GPS stream...")

        while self.running:
            # grap each package to clear the buffer
            self.gps.next()

    def stop(self):
        self.running = False

    def read(self):
        return GPSState(self.gps)

    def update(self):
        try:
            report = self.gps.next()

            # Wait for a 'TPV' report and display the current time
            # To see all report data, uncomment the line below
            self.log.debug(report)
            if report['class'] == 'TPV':
                if hasattr(report, 'time'):
                    self.log.debug("time:" + report.time)
        except KeyError:
            self.log.debug("KeyError")
            pass
        except KeyboardInterrupt:
            self.log.error("KeyboardInterrupt!")
            quit()
        except StopIteration:
            session = None
            self.log.error("GPSD has terminated")



