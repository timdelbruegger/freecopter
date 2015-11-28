from axis_speeds import AxisSpeeds
from state_change_planning import State

TIME_BETWEEN_CONTROL_LOOP_UPDATES = 0.02 # 50 Hz
TIME_TO_REACH_TARGET_STATE = 0.2

# Responsible for keeping the Quadrocopter in the air
# by following some given flight plan
class ControlLoop:

    def __init__(self, motors, pidclass):
        self.motors = motors
        self.xRotPID = pidclass()
        self.yRotPID = pidclass()
        self.zRotPID = pidclass()
        self.vertSpdPID = pidclass()

    # Do one loop iteration
    # - time_delta:    seconds since last update
    # - quadro_state:  current state from sensor fusion
    # - quadro_target: the target state (from high-level path planning)
    #                  that we should reach within TIME_TO_REACH_TARGET_STATE seconds
    def step(self, time_delta, quadro_state, quadro_target):

        #--------------------------------------------------
        # Axis rotation planning for all 3 axis
        rotPlans = quadro_state.rotation.planChangeTo(quadro_target.rotation).inSeconds(TIME_TO_REACH_TARGET_STATE)

        #--------------------------------------------------
        # do elevation planning for altitude
        elevationPlan = quadro_state.elevation.planChangeTo(quadro_target.elevation).inSeconds(TIME_TO_REACH_TARGET_STATE)

        #--------------------------------------------------
        # Prepare inputs for PIDs

        # calculate the target rotation speeds for the very next update
        targetRotSpeeds = map(
            lambda plan: plan.calculateTargetSpeedForTimePeriod(0,TIME_BETWEEN_CONTROL_LOOP_UPDATES),
            rotPlans
        )
        # calculate the target elevation speed for the very next update
        targetElevationSpeed = elevationPlan.calculateTargetSpeedForTimePeriod(0,TIME_BETWEEN_CONTROL_LOOP_UPDATES)

        #--------------------------------------------------
        # Update PIDs
        # At the moment we only use the first plan segments above, because we always
        # do a replanning. We could break the control loop into two loops:
        # fast PIDs and slower state change planning. Then, also the other plan
        # segments would be used. For example, a replanning could be done
        # every 0.1 seconds and the PIDs could adjust the motors every 0.01 seconds.
        axisSpeeds = AxisSpeeds()
        axisSpeeds.xAxisRotationSpeed = self.xRotPID.update(rotStates(0), targetRotSpeeds(0), time_delta)
        axisSpeeds.yAxisRotationSpeed = self.yRotPID.update(rotStates(1), targetRotSpeeds(1), time_delta)
        axisSpeeds.zAxisRotationSpeed = self.zRotPID.update(rotStates(2), targetRotSpeeds(2), time_delta)
        axisSpeeds.verticalSpeed = self.vertSpdPID.update(elevationState, targetElevationSpeed, time_delta)

        # transform to motor speeds and send to motors
        self.motors.setSpeed(axisSpeeds.toMotorSignals())

        
