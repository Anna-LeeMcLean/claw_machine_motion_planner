 #!/usr/bin/env python

import sys
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from motion_planner.utils import Prize
from motion_planner.motion_state import StartState, DropoffStandoffState
from motion_planner.motion_plan import MotionPlan


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


    def parse_and_sort_prize_data(self):
        '''
            Sorts prize data in order from highest in the bin to lowest in the bin and appends 
            custom prize objects to a global list in said order.
        '''

        z_values = [prize["position"]["z"] for prize in self.prize_data]

        z_values.sort(reverse=True)

        for value in z_values:
            for i in range(len(self.prize_data)):
                if self.prize_data[i]["position"]["z"] == value:
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

        starting_state = StartState()
        ending_state = DropoffStandoffState()

        for prize in self.prize_objects:

            self.get_logger().info(f"{prize = }")
            prize_planner = MotionPlan(prize=prize, starting_state=starting_state)

            prize_planner.plan_states()

            states = [state.convert_to_dict() for state in prize_planner.states]

            step_data = {"states": states,
                        "total_elapsed_time": prize_planner.total_elapsed_time,
                        "prize_picked": prize.index}
            
            self.get_logger().info(f"{step_data = }")

            self._make_json_object(data=step_data)

            starting_state = ending_state


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
    
    node.parse_and_sort_prize_data()

    node.plan_steps()

    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
