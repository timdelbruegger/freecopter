import numpy

# Own implementation of a PID controller. Ideas based on the Arduino code from
# Brett Beauregard: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
# - Derivative Kick
# - Reset window mitigation
class PIDController():

    def __init__(self, kp, ki, kd, state, minOutput, maxOutput):
        self.kp = kp # proportional coefficient
        self.ki = ki # integral coefficient
        self.kd = kd # differential coefficient^
        self.time = 0 # absolute point in time in seconds
        self.errorSum = 0 # sum of all errors
        self.lastState = state # current state of the system that we want to control
        self.minOutput = minOutput
        self.maxOutput = maxOutput

    # timeDelta: seconds since last compute
    def compute(self, currentState, targetState, timeDelta):

        error = targetState - currentState
        self.errorSum += error * timeDelta

        # This seems to eliminate lags when the integral component goes beyond
        # the output limits and needs time to get back later.
        iTerm = self._limitOutput(self.errorSum * self.ki)

        # yes, this is "Derivative on Input":
        # dErr = (self.lastState - currentState) / timeDelta
        dErr = ((targetState - currentState) - (targetState - self.lastState)) / timeDelta

        # compute output
        output = self.kp * error + iTerm + self.kd * dErr

        # obey limits
        output = self._limitOutput(output)

        # save values for next iteration
        self.time += timeDelta
        self.lastState = currentState

        return output

    def _limitOutput(self, x):
        return max(self.minOutput, min(x, self.maxOutput))
