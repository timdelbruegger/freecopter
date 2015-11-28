from state_provider import StateProvider

# Responsible for giving targets to the Quadrocopter control lopp
class HighLevelLogic:

    def __init__(self):
        self.controllers
        self.flightMode = FlightModeLanded()
        self.controlLoop = controlLoop(motors, PID)
        self.stateProvider = FusedStateProvider()
        
        self.stateProvider.registerListener(self)

    # Tells all systems that time has passed.
    # timeDelta is the time since the last update call in seconds
    def update(self, timeDelta):
        self.flightMode.update(timeDelta)
        self.stateProvider.update(timeDelta)
        
    # will be called by stateProvider whenever a new sensor reading is ready
    # timeDelta is the time since the last newState call in seconds
    def newState(self, timeDelta, state):
        targetState = self.flightMode.calculateTargetState(state)
        self.controlLoop.setTargetState(targetState)
        
class FlightState:

    def __init__(self):
        self.timeInState = 0

    def update(self, timeDelta):
        self.timeInState += timeDelta
        return self._update(timeDelta)

class FlightStateLanded(FlightState):

    def __init__(self):
        pass

    def _update(self, timeDelta):
        if self.timeInState > 2.0:
            return FlightStateRiseTo1m()

        return self
        
    def calculateTargetState(self, )

class FlightStateRiseTo1m(FlightState):
    def __init__(self):
        # start motors


    def _update(self, timeDelta):
        pass

class FlightStateHover(FlightState):
    def __init__(self):
        pass

    def _update(self, timeDelta):
        pass


class FlightStateGoDown(FlightState):
    def __init__(self):
        pass

    def _update(self, timeDelta):
        pass
