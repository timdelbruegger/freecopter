from state_change_planning import State


# 3 axis version of StateChangeSelectTimeToReachTarget. It just does one planning per axis
class State3dChangeSelectTimeToReachTarget:
    def __init__(self, current_states, target_states):
        self.currentStates = current_states
        self.targetStates = target_states

    def in_seconds(self, time_to_reach_target_state):

        def plan_change_in_seconds (state_target_time):
            state, target, time = state_target_time
            return state.plan_change_to(target).in_seconds(time)
        plans = map(
            plan_change_in_seconds,
            zip(self.currentStates, self.targetStates, [time_to_reach_target_state] * len(self.currentStates))
        )

        return StateChangePlanMultiDim(plans)


# One plan for every dimension
class StateChangePlanMultiDim:
    def __init__(self, plans):
        self.plans = plans

    # In the time period (t_begin, t_end), we want to achieve this target speed
    # t_begin and t_end are in real physical seconds. The polynome has a different domain (timeunit)
    def calculate_target_speed_for_time_period(self, t_begin, t_end):

        return list(map((lambda plan: plan.calculate_target_speed_for_time_period(t_begin, t_end)), self.plans))


# A 3-dimensional state includes a value and a speed for every axis (x, y, z).
# It serves as an entry point for StateChangePlanning in 3d.
class State3d:

    def __init__(self):
        self.x = State.by_value_and_speed(0, 0)
        self.y = State.by_value_and_speed(0, 0)
        self.z = State.by_value_and_speed(0, 0)

    def plan_change_to(self, target):
        xyz_states = [self.x, self.y, self.z]
        xyz_targets = [target.x, target.y, target.z]
        return State3dChangeSelectTimeToReachTarget(xyz_states, xyz_targets)
