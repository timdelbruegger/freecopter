
import logging
from util.timer import Timer

class LoggingStateProviderWithListeners:

    def __init__(self, name):
        self.log = logging.getLogger(name)
        self.__listeners = []
        self.__timer = Timer()

    def registerListener(self, listener):
        self.log.debug("Appending listener: " + listener.name)
        self.__listeners.append(listener)

    # Notifies all listeners after the state changed. Measures the time between updates to give it to listeners.
    # Should be called by the child class.
    def notify_listeners(self, newstate):

        # newstate = self._read_state(self.imu.getIMUData())

        # new state is ready, we can print it
        self.log.debug("Notify listeners of new state")
        self.log.debug(newstate)

        # give new state to listeners
        time_since_last_update = self.__timer.readAndReset()
        for listener in self.__listeners:
            listener.new_state(time_since_last_update, newstate)
