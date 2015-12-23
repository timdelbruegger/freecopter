import numpy;
from numpy.polynomial.polynomial import Polynomial, polyval

# The concepts of this file were originally published as Axis Rotation Planning:
# https://timdelbruegger.wordpress.com/2015/09/26/quadrocopter-control-loop
# However, the same principle can be used to plan a change in the height of the
# quadrocopter, so the new name "State Change Planning" is more appropriate.

class StateChangeSelectTimeToReachTarget:
    def __init__(self, current, target):
        self._current = current
        self._target = target

    def in_seconds(self, time_to_reach_target_state):

        curVal = self._current.value
        curSpd = self._current.speed
        tarVal = self._target.value
        tarSpd = self._target.speed

        a =  2*curVal +   curSpd - 2*tarVal + tarSpd
        b = -3*curVal - 2*curSpd + 3*tarVal - tarSpd
        c =               curSpd
        d =    curVal
        plan = StateChangePlan(a, b, c, d, time_to_reach_target_state)

        return plan


class State:
    def __init__(self):
        self.value = 0.0 # the current value
        self.speed = 0.0 # the speed with which the value changes over time

    @staticmethod
    def by_value_and_speed(value, speed):
        state = State()
        state.value = value
        state.speed = speed
        return state

    def plan_change_to(self, target):
        return StateChangeSelectTimeToReachTarget(self, target)


# A polynomial of degree 3: ax^3+bx^2+cx+d
# This polynomial represents the rotation state around one axis or a change in
# the height of the quadrocopter. The derivative of this polynomial is the
# rotation speed / elevation speed.
# timeToReachTargetState = 1.3 would mean the target state is reached after 1.3 seconds.
class StateChangePlan:
    def __init__(self, a, b, c, d, time_to_reach_target_state):
        coef = [d, c, b, a]
        self._plan = Polynomial(coef)
        self._timeunit = time_to_reach_target_state
#        print('StateChangePlan.coef: ' + str(coef))
#        print('StateChangePlan.plan: ' + str(self._plan))
#        print('StateChangePlan.timeunit: ' + str(self._timeunit))

    # calculates the value at some given time in the future
    def valueAt(self, t):
        #   coef = [self.d, self.c, self.b, self.a]
        #   return polyval(t/self.timeunit, coef)
        return self._plan(t/self._timeunit)

    # In the time period (t_begin, t_end), we want to achieve this target speed
    # t_begin and t_end are in real physical seconds. The polynome has a different domain (timeunit)
    def calculate_target_speed_for_time_period(self, tbegin, tend):

        return (self.valueAt(tend) - self.valueAt(tbegin)) / self._timeunit
