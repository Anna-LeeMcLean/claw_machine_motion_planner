from dataclasses import dataclass
from geometry_msgs.msg import Point

@dataclass
class Prize:
    def __init__(self, centroid:Point, bounding_box:Point, index:int) -> None:
        self.index = index
        self.centroid = centroid
        self.bounding_box = bounding_box

