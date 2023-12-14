from dataclasses import dataclass, field
from geometry_msgs.msg import Point

@dataclass
class Prize:

    index: int = field(repr=False)
    centroid: Point
    bounding_box: Point
