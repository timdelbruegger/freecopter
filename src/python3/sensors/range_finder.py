# import required modules
import time
import RPi.GPIO as GPIO
from state_change_planning import State

# When calculating the new distance, we use a weighted average between the last value and the new measurement (low pass filtering).
# FILTER_STRENGTH is the weight of the last value, the new value has the weight (1-FILTER_STRENGTH)
VALUE_FILTER_STRENGTH = 0.7


# This code works with a HC-SR04 ultrasonic range finder.
# See http://www.modmypi.com/blog/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi
class UltrasonicRangeFinder:

    def __init__(self, gpio_trigger, gpio_echo):

        # define GPIO pins
        self.gpio_trigger = gpio_trigger
        self.gpio_echo = gpio_echo
        self.distance = None
        self.speed = None

        # use GPIO pin numbering convention
        GPIO.setmode(GPIO.BCM)

        # set up GPIO pins
        GPIO.setup(self.gpio_trigger, GPIO.OUT)
        GPIO.setup(self.gpio_echo, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # set trigger to false
        GPIO.output(self.gpio_trigger, False)

    # function to measure the distance in meters
    def _measure_distance(self):
        # set trigger to high
        GPIO.output(self.gpio_trigger, True)

        # set trigger after 10µs to low
        time.sleep(0.00001)
        GPIO.output(self.gpio_trigger, False)

        # store initial start time
        time_start = time.time()

        # store start time
        #if GPIO.input(self.gpio_echo) == 0:
        #print("Waiting for rising Echo edge")
        #    GPIO.wait_for_edge(self.gpio_echo, GPIO.RISING)
        while GPIO.input(self.gpio_echo) == 0:
            time_start = time.time()

        # store stop time
        # this time span is proportionate to the distance of an obstacle
        # if there is no echo (= no obstacle), we this will take 38 ms here.
        time_stop = time.time()
        #if GPIO.input(self.gpio_echo) == 1:
        #print("Waiting for falling Echo edge")
        # GPIO.wait_for_edge(self.gpio_echo, GPIO.FALLING)
        while GPIO.input(self.gpio_echo) == 1:
            time_stop = time.time()

        # calculate distance
        time_elapsed = time_stop - time_start
        distance_now_cm = time_elapsed * 34300 / 2

        distance_now_meters = distance_now_cm * 0.01
        return distance_now_meters

    # infinite loop that puts new distance values into the given queue.
    # Stops as soon as
    def measure(self, queue, stop_flag, time_between_measurements):

        try:
            time_last_update = time.time()
            while not stop_flag.value:

                    distance_now = self._measure_distance()
                    time_now = time.time()
                    duration_measurement = time_now-time_last_update
                    time_last_update = time_now

                    # use weighted average as filter
                    if self.distance is None:
                        self.distance = distance_now
                        self.speed = 0.0
                    else:
                        newfiltereddistance = self.distance * VALUE_FILTER_STRENGTH + distance_now * (1 - VALUE_FILTER_STRENGTH)
                        self.speed = (newfiltereddistance - self.distance) / duration_measurement
                        self.distance = newfiltereddistance

                    print("Measured Distance = %.12f m" % distance_now)
                    print("Filtered Distance = %.12f m" % self.distance)

                    queue.put(State.by_value_and_speed(self.distance, self.speed))

                    time.sleep(time_between_measurements)
        finally:
            # after we stop measuring, we should clean up
            # TODO: is this really right? Maybe someone else is still active on GPIO?
            # should not be the case, we are in our own process
            GPIO.cleanup()