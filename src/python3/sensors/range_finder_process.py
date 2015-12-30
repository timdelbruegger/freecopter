from multiprocessing import Process, Queue, Value
from sensors.range_finder import UltrasonicRangeFinder

time_between_measurements = 0.001


def _range_finder_process(gpio_trigger, gpio_echo, queue, stop_flag):

    rangefinder = UltrasonicRangeFinder(gpio_trigger, gpio_echo)
    rangefinder.measure(queue, stop_flag, time_between_measurements)


# Because the ultrasonic range finder uses busy waiting on an echo, we don't want
# to block the execution thread and rather use a second process. This class wraps
# the process so that it can be used as if it were the range finder directly.
class UltrasonicRangeFinderProcess:

    def __init__(self, gpio_trigger, gpio_echo):
        self.distance_queue = Queue()
        self.stop_flag = Value('i', False)
        self.process = Process(
                target=_range_finder_process,
                args=(gpio_trigger, gpio_echo, self.distance_queue, self.stop_flag))

        self.distance = None

    def start(self):
        self.process.start()

    def stop(self):
        self.stop_flag.value = True
        self.process.join()

    def read_distance(self):

        while not self.distance_queue.empty():
            self.distance = self.distance_queue.get(False)

        return self.distance