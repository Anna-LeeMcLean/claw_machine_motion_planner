from math import ceil
from geometry_msgs.msg import Point

CLAW_SPEED_M_PER_S = 0.2        # 20cm/s -> 0.2m/s
CLAW_ACTIVATION_TIME = 1        # 1 sec to open/close claw

class Prize:
    def __init__(self, centroid:Point, bounding_box:Point, index:int) -> None:
        self.index = index
        self.centroid = centroid
        self.bounding_box = bounding_box

    def __repr__(self) -> str:
        return f"centroid: {self.centroid},   index: {self.index}"

class State:
    def __init__(self, position: Point) -> None:
        self.position = position
        self.claw_state = False     # true if claw is open and false otherwise
        self.elapsed_time = 0

    def set_claw_state(self, value:bool):
        self.claw_state = value

    def set_elapsed_time(self, starting_point:Point, claw_activated = False):
        travel_distances = [abs(self.position.x - starting_point.x),
                            abs(self.position.y - starting_point.y),
                            abs(self.position.z - starting_point.z)]

        travel_times = [(1/CLAW_SPEED_M_PER_S) * distance for distance in travel_distances]

        total = 0
        for time in travel_times:
            total += time

        self.elapsed_time = ceil(total)    # round up to the nearest integer

        if claw_activated:
            self.elapsed_time += CLAW_ACTIVATION_TIME

    def convert_to_dict(self) -> dict:

        state_dict = {"position": {"x":self.position.x,
                                   "y":self.position.y,
                                   "z":self.position.z},
                      "claw_state": self.claw_state,
                      "elapsed_time": self.elapsed_time}

        return state_dict
