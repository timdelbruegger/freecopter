from datetime import datetime
from pid import PIDController
from motors import AdafruitPwmControlledMotors
from state_provider import StateProvider
from high_level_logic import HighLevelLogic
from controlLoop import ControlLoop
import logging


# The Initiator binds the various freecopter modules together, following the inversion of control pattern.
class Initiator:

    # Here we wire all the components together
    def __init__(self):
        logging.basicConfig(filename='freecopter.log', level=logging.INFO)

        self.motors = AdafruitPwmControlledMotors(debug_flag=True)
        self.controlLoop = ControlLoop(self.motors, PIDController)
        self.stateProvider = StateProvider()
        self.highLevelLogic = HighLevelLogic(self.controlLogic)

        # Will be set on the first update.
        self.time_of_last_update = None

    def update(self):

        now = datetime.now()

        if self.time_of_last_update is None:
            # some very small time Delta for the start
            time_delta = 0.000001
        else:
            time_delta = (now - self.time_of_last_update).total_seconds()

        # gather sensor information
        self.stateProvider.update()

        # high level logic updates the other systems
        self.highLevelLogic.update(time_delta)

        # save time for next iteration
        self.time_of_last_update = now

if __name__ == '__main__':
    initiator = Initiator()
