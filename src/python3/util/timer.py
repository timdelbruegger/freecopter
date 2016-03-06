from datetime import datetime


# Helper class to quickly get the time since last update / last method invocation
class Timer:

    def __init__(self):
        self.__lastUpdate = 0
        self.reset()

    def reset(self):
        self.__lastUpdate = datetime.now()

    def readAndReset(self):
        time_since_last_reset = (datetime.now() - self.__lastUpdate).total_seconds()
        self.reset()
        return time_since_last_reset
