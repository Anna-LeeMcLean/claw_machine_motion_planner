 #!/usr/bin/env python

import sys
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from motion_planner.motion_plan import MotionPlan, Prize


class PlanMotionSteps(Node):

    def __init__(self, json_path, json_file_name) -> None:
        super().__init__(node_name="plan_motion_steps", namespace="/planner")
        self.json_path = json_path
        self.json_file_name = json_file_name
        self.prize_data = []
        self.prize_objects: list[Prize] = []

        full_path = self.json_path + self.json_file_name

        with open(full_path, 'r') as f:
            prize_dict = json.load(f)
            self.prize_data = prize_dict["prizes"]
        
        self.get_logger().info(f"{self.prize_data = }")

        for i in range(len(self.prize_data)):
            prize_object = self.make_prize_object(self.prize_data[i], i)
            self.prize_objects.append(prize_object)


    def make_prize_object(self, prize_data : dict, index):
        centroid = Point(x = prize_data["position"]["x"],
                        y = prize_data["position"]["y"],
                        z = prize_data["position"]["z"])
        
        bounding_box = Point(x = prize_data["bounding_box"]["x"],
                            y = prize_data["bounding_box"]["y"],
                            z = prize_data["bounding_box"]["z"])
        
        prize = Prize(centroid=centroid, bounding_box=bounding_box, index=index)
        return prize
    

    def plan_steps(self):
        for prize in self.prize_objects:

            self.get_logger().info(f"{prize = }")
            prize_planner = MotionPlan(prize=prize)

            prize_planner.plan_states()

            states = [state.convert_to_dict() for state in prize_planner.states]

            step_data = {"states": states,
                        "total_elapsed_time": prize_planner.total_elapsed_time,
                        "prize_picked": prize.index}
            
            self.get_logger().info(f"{step_data = }")

            self._make_json_object(data=step_data)


    def _make_json_object(self, data: dict):
        json_file_name = f'step_data_prize_{data["prize_picked"]}.json'

        full_path = self.json_path + json_file_name

        with open(full_path, "w") as f:
            json.dump(data, f)
    

def main(args=None):

    if args is None:
        args = sys.argv
    
    rclpy.init(args=args)

    json_file_path = args[1]
    json_file_name = args[2]
    
    node = PlanMotionSteps(json_path=json_file_path, json_file_name=json_file_name)

    node.plan_steps()

    
if __name__ == '__main__':
    main()