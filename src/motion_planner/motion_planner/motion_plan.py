from motion_planner.utils import Prize
from motion_planner.motion_state import MotionState, StartState


class MotionPlan():

    def __init__(self, prize:Prize) -> None:

        self._total_elapsed_time = 0 
        self.prize = prize
        self._states : list[MotionState] = []
    
    @property
    def states(self):
        return self._states
    
    @property
    def total_elapsed_time(self):
        return self._total_elapsed_time

    
    def plan_states(self):

        MotionState.prize = self.prize

        current_state = StartState()
        next_state = current_state.get_next_motion_state()
        
        while next_state != None:
            current_state.set_position()
            current_state.set_claw_state()
            current_state.set_elapsed_time()
            self._states.append(current_state)
            next_state = current_state.get_next_motion_state()
            current_state = next_state


        self._update_total_elapsed_time()


    def _update_total_elapsed_time(self):
        total = 0
        for i in range(len(self._states)):
            total += self._states[i].elapsed_time

        self._total_elapsed_time = total
