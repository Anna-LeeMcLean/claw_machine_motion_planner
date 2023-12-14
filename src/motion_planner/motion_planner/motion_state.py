from math import ceil
from abc import ABC, abstractmethod

from motion_planner.utils import Prize

from geometry_msgs.msg import Point

CLAW_SPEED_M_PER_S = 0.2        # 20cm/s -> 0.2m/s
CLAW_ACTIVATION_TIME = 1        # 1 sec to open/close claw

MAX_CLAW_HEIGHT_M = 1.8         # 180cm in world z axis
STARTING_POINT_X = 0.5          # 50cm in world x axis
STARTING_POINT_Y = 0.5          # 50cm in world y axis
DROPOFF_POINT_X = 0.05          # 5cm in world x axis
DROPOFF_POINT_Y = 0.05          # 5cm in world y axis
DROPOFF_POINT_Z = 0.5           # 50cm in world z axis


class MotionState(ABC):

    prize: Prize

    def __init__(self, prev_motion_state = None) -> None:
        super().__init__()
        self.position: Point = None
        self.claw_state: bool = None     # true if claw is open and false otherwise
        self.elapsed_time: int = None
        self.previous_motion_state: MotionState = prev_motion_state
        self.next_motion_state: MotionState = None

    @abstractmethod
    def set_elapsed_time(self):
        pass

    @abstractmethod
    def set_claw_state(self):
        pass

    @abstractmethod
    def set_position(self):
        pass

    @abstractmethod
    def get_next_motion_state(self):
        pass

    def convert_to_dict(self) -> dict:

        state_dict = {"position": {"x":self.position.x,
                                   "y":self.position.y,
                                   "z":self.position.z},
                      "claw_state": self.claw_state,
                      "elapsed_time": self.elapsed_time}

        return state_dict


class StartState(MotionState):

    def __init__(self, prev_motion_state=None) -> None:
        super().__init__(prev_motion_state)

        self.next_motion_state = SetClawState(prev_motion_state=self)

    def set_elapsed_time(self):
        self.elapsed_time = 0

    def set_claw_state(self):
        self.claw_state = False

    def set_position(self):
        self.position = Point(x=STARTING_POINT_X, 
                              y=STARTING_POINT_Y, 
                              z=MAX_CLAW_HEIGHT_M)

    def get_next_motion_state(self):
        return self.next_motion_state


class SetClawState(MotionState):

    def __init__(self, prev_motion_state=None) -> None:
        super().__init__(prev_motion_state)

    def set_elapsed_time(self):
        self.elapsed_time = CLAW_ACTIVATION_TIME

    def set_claw_state(self):
        if self.previous_motion_state.claw_state:
            self.claw_state = False
        else:
            self.claw_state = True

    def set_position(self):
        self.position = Point(x=self.previous_motion_state.position.x, 
                              y=self.previous_motion_state.position.y, 
                              z=self.previous_motion_state.position.z)

    def get_next_motion_state(self):
        if isinstance(self.previous_motion_state, DropoffCentroidState):
            self.next_motion_state = DropoffStandoffState(prev_motion_state=self)
        else:
            self.next_motion_state = PrizeStandoffState(prev_motion_state=self)

        return self.next_motion_state


class TranslationState(MotionState):

    def __init__(self, prev_motion_state=None) -> None:
        super().__init__(prev_motion_state)

    def set_elapsed_time(self):
        travel_distances = [abs(self.position.x - self.previous_motion_state.position.x),
                            abs(self.position.y - self.previous_motion_state.position.y),
                            abs(self.position.z - self.previous_motion_state.position.z)]

        travel_times = [(1/CLAW_SPEED_M_PER_S) * distance for distance in travel_distances]

        total = 0
        for time in travel_times:
            total += time

        self.elapsed_time = ceil(total)    # round up to the nearest integer

    def set_claw_state(self):
        self.claw_state = self.previous_motion_state.claw_state

    def set_position(self):
        return super().set_position()

    def get_next_motion_state(self):
        return super().get_next_motion_state()


class PrizeCentroidState(TranslationState):

    def __init__(self, prev_motion_state=None) -> None:
        super().__init__(prev_motion_state)

    def set_elapsed_time(self):
        return super().set_elapsed_time()

    def set_claw_state(self):
        return super().set_claw_state()
    
    def set_position(self):
        self.position = Point(x=self.prize.centroid.x, 
                              y=self.prize.centroid.y, 
                              z=self.prize.centroid.z)
    
    def get_next_motion_state(self) -> MotionState:
        
        self.next_motion_state = SetClawState(prev_motion_state=self)

        return self.next_motion_state


class PrizeStandoffState(TranslationState):

    def __init__(self, prev_motion_state=None) -> None:
        super().__init__(prev_motion_state)

    def set_elapsed_time(self):
        return super().set_elapsed_time()

    def set_claw_state(self):
        return super().set_claw_state()
    
    def set_position(self):
        self.position = Point(x=self.prize.centroid.x, 
                              y=self.prize.centroid.y, 
                              z=MAX_CLAW_HEIGHT_M)
    
    def get_next_motion_state(self) -> MotionState:

        if self.previous_motion_state.claw_state:
            self.next_motion_state = PrizeCentroidState(prev_motion_state=self)
        else:
            self.next_motion_state = DropoffStandoffState(prev_motion_state=self)

        return self.next_motion_state


class DropoffCentroidState(TranslationState):

    def __init__(self, prev_motion_state=None) -> None:
        super().__init__(prev_motion_state)

    def set_elapsed_time(self):
        return super().set_elapsed_time()

    def set_claw_state(self):
        return super().set_claw_state()
    
    def set_position(self):
        self.position = Point(x=DROPOFF_POINT_X, 
                              y=DROPOFF_POINT_Y, 
                              z=DROPOFF_POINT_Z)
    
    def get_next_motion_state(self) -> MotionState:

        self.next_motion_state = SetClawState(prev_motion_state=self)

        return self.next_motion_state
    

class DropoffStandoffState(TranslationState):

    def __init__(self, prev_motion_state=None) -> None:
        super().__init__(prev_motion_state)

    def set_elapsed_time(self):
        if self.previous_motion_state is None:
            # Restarting a new motion plan
            self.elapsed_time = 0
        else:
            super().set_elapsed_time()

    def set_claw_state(self):
        if self.previous_motion_state is None:
            # Restarting a new motion plan
            self.claw_state = True
        else:
            super().set_claw_state()
    
    def set_position(self):
        self.position = Point(x=DROPOFF_POINT_X, 
                              y=DROPOFF_POINT_Y, 
                              z=MAX_CLAW_HEIGHT_M)
    
    def get_next_motion_state(self) -> MotionState:

        if self.previous_motion_state is None:
            # Restarting a new motion plan
            self.next_motion_state = PrizeStandoffState(prev_motion_state=self)
        elif self.previous_motion_state.claw_state:
            self.next_motion_state = None
        else:
            self.next_motion_state = DropoffCentroidState(prev_motion_state=self)

        return self.next_motion_state
    