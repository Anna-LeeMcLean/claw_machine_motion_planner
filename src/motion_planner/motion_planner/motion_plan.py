from geometry_msgs.msg import Point
from motion_planner.utils import Prize, State

MAX_CLAW_HEIGHT_M = 1.8         # 180cm in world z axis
STARTING_POINT_X = 0.5          # 50cm in world x axis
STARTING_POINT_Y = 0.5          # 50cm in world y axis
DROPOFF_POINT_X = 0.05          # 5cm in world x axis
DROPOFF_POINT_Y = 0.05          # 5cm in world y axis
DROPOFF_POINT_Z = 0.5           # 50cm in world z axis


class MotionPlan():
    starting_point = Point(x=STARTING_POINT_X, y=STARTING_POINT_Y, z=MAX_CLAW_HEIGHT_M)
    dropoff_point = Point(x=DROPOFF_POINT_X, y=DROPOFF_POINT_Y, z=DROPOFF_POINT_Z)

    def __init__(self, prize:Prize) -> None:

        self._total_elapsed_time = 0 
        self.prize = prize
        self._states : list[State] = []
    
    @property
    def states(self):
        return self._states
    
    @property
    def total_elapsed_time(self):
        return self._total_elapsed_time

    def _get_prize_standoff_point(self) -> Point:

        prize_standoff = Point()
        prize_standoff.x = self.prize.centroid.x
        prize_standoff.y = self.prize.centroid.y
        prize_standoff.z = MAX_CLAW_HEIGHT_M

        return prize_standoff
    
    def _get_dropoff_standoff_point(self) -> Point:

        dropoff_standoff = Point()
        dropoff_standoff.x = self.dropoff_point.x
        dropoff_standoff.y = self.dropoff_point.y
        dropoff_standoff.z = MAX_CLAW_HEIGHT_M

        return dropoff_standoff
    
    def plan_states(self):

        # 1. Starting State
        starting_state = State(position=self.starting_point)
        self._states.append(starting_state)

        # 2. Prize Standoff State (180cm above the prize)
        prize_standoff_point = self._get_prize_standoff_point()
        prize_standoff_state = State(position=prize_standoff_point)
        prize_standoff_state.set_elapsed_time(starting_point=starting_state.position)
        self._states.append(prize_standoff_state)

        # 3. Open Claw
        open_claw_state = State(position=prize_standoff_state.position)
        open_claw_state.set_elapsed_time(starting_point=open_claw_state.position, claw_activated=True)
        open_claw_state.set_claw_state(value=True)
        self._states.append(open_claw_state)

        # 4. Go to Prize
        go_to_prize_state = State(position=self.prize.centroid)
        go_to_prize_state.set_elapsed_time(starting_point=open_claw_state.position)
        go_to_prize_state.set_claw_state(value=True)
        self._states.append(go_to_prize_state)

        # 5. Close Claw
        close_claw_state = State(position=self.prize.centroid)
        close_claw_state.set_elapsed_time(starting_point=go_to_prize_state.position, claw_activated=True)
        self._states.append(close_claw_state)

        # 6. Prize Standoff State (180cm above the prize)
        prize_standoff_point = self._get_prize_standoff_point()
        prize_standoff_state = State(position=prize_standoff_point)
        prize_standoff_state.set_elapsed_time(starting_point=go_to_prize_state.position)
        self._states.append(prize_standoff_state)

        # 7. Dropoff Standoff State (180cm above the dropoff)
        dropoff_standoff_point = self._get_dropoff_standoff_point()
        dropoff_standoff_state = State(position=dropoff_standoff_point)
        dropoff_standoff_state.set_elapsed_time(starting_point=prize_standoff_state.position)
        self._states.append(dropoff_standoff_state)

        # 8. Go to Dropoff
        go_to_dropoff_state = State(position=self.dropoff_point)
        go_to_dropoff_state.set_elapsed_time(starting_point=dropoff_standoff_state.position)
        self._states.append(go_to_dropoff_state)

        # 9. Open Claw
        open_claw_state = State(position=go_to_dropoff_state.position)
        open_claw_state.set_elapsed_time(starting_point=open_claw_state.position, claw_activated=True)
        open_claw_state.set_claw_state(value=True)
        self._states.append(open_claw_state)

        # 10. Dropoff Standoff State (180cm above the dropoff)
        dropoff_standoff_point = self._get_dropoff_standoff_point()
        dropoff_standoff_state = State(position=dropoff_standoff_point)
        dropoff_standoff_state.set_elapsed_time(starting_point=open_claw_state.position)
        dropoff_standoff_state.set_claw_state(value=True)
        self._states.append(dropoff_standoff_state)

        # 11. Close Claw
        close_claw_state = State(position=dropoff_standoff_state.position)
        close_claw_state.set_elapsed_time(starting_point=close_claw_state.position, claw_activated=True)
        self._states.append(close_claw_state)

        # 12. Back to Starting Point
        back_to_start_state = State(position=self.starting_point)
        back_to_start_state.set_elapsed_time(starting_point=close_claw_state.position)
        self._states.append(back_to_start_state)


        self._update_total_elapsed_time()


    def _update_total_elapsed_time(self):
        total = 0
        for i in range(len(self._states)):
            total += self._states[i].elapsed_time

        self._total_elapsed_time = total

        
