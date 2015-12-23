# Responsible for giving targets to the Quadrocopter control lopp
class HighLevelLogic:

    def __init__(self, control_loop, state_provider):
        self.controllers
        self.flightmode = FlightModeLanded()
        self.control_loop = control_loop

        state_provider.registerListener(self)

    # Tells all systems that time has passed.
    # timeDelta is the time since the last update call in seconds
    def update(self, timedelta):

        # check if we need to change the flight mode
        newmode = self.flightmode.update(timedelta)

        if newmode.name != self.flightmode.name:
            print("HighLevelLogic: changing FlightMode from %s to %s" % (self.flightmode.name, newmode.name))
            self.flightmode = newmode

        # TODO: do we need to update the control loop?

    # will be called by State Provider whenever a new sensor reading is ready
    # timeDelta is the time since the last newState call in seconds
    def new_sensor_reading(self, timedelta, state):
        target_state = self.flightmode.calculate_target_state(state)
        self.control_loop.setTargetState(target_state)


class FlightMode:

    def __init__(self):
        self.timeInState = 0

    def update(self, time_delta):
        self.timeInState += time_delta
        return self._update(time_delta)


class FlightModeLanded(FlightMode):

    def __init__(self):
        self.name = "FlightModeLanded"

    def _update(self, timedelta):
        if self.timeInState > 2.0:
            return FlightModeRiseTo1m()

        return self

    def calculate_target_state(self, current_state):

        # no need to react to anything
        return self


class FlightModeRiseTo1m(FlightMode):
    def __init__(self):
        # TODO: start motors
        self.name = "FlightModeRiseTo1m"

    def _update(self, timedelta):
        if self.timeInState > 3.0:
            return FlightModeHover()

        return self

    def calculate_target_state(self, current_state):
        # no need to react to anything
        return self


class FlightModeHover(FlightMode):
    def __init__(self):
        self.name = "FlightModeHover"

    def _update(self, timedelta):
        if self.timeInState > 5.0:
            return FlightModeGoDown()

        return self

    def calculate_target_state(self, current_state):
        # no need to react to anything
        return self


class FlightModeGoDown(FlightMode):
    def __init__(self):
        self.name = "FlightModeGoDown"

    def _update(self, timedelta):
        if self.timeInState > 7.0:
            return FlightModeLanded()

        return self

    def calculate_target_state(self, current_state):
        # no need to react to anything
        return self

