 #!/usr/bin/env python
''' Load and parse states then request their execution '''

import os
import sys
import json
import random
import numpy as np
from queue import SimpleQueue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

OPEN_CLAW_DISTANCE = 0.3                    # width between claw fingers. this is the travel distance needed to fully open the claw based on the URDF
CLOSED_CLAW_DISTANCE = 0.0
CLAW_ACTIVATION_TIME = 1.0                  # time taken to fully open/close claw in seconds
CLAW_SPEED_M_PER_S = 0.2                    # claw speed in m/s (NOTE: this is the speed of the links which move the claw and not the speed at which the claw opens/closes)
PUBLISH_RATE = 10                           # rate at which joint state updates are sent on the /joint_states topic
LINK_STEP_M = CLAW_SPEED_M_PER_S/PUBLISH_RATE    # interpolation distance needed to update joint positions so that translation occurs at joint speed
CLAW_STEP_M = OPEN_CLAW_DISTANCE/(CLAW_ACTIVATION_TIME * PUBLISH_RATE)    # interpolation distance needed to update claw position so that translation takes 1 second

class VisualizePrizes(Node):
    def __init__(self, json_path) -> None:
        super().__init__("visualize_prizes")

        # Get json data
        self.prize_data = []

        # Parse json data and populate the two attributes above
        self._parse_json_data(json_path=json_path)

        self.get_logger().info(f"{self.prize_data = }")

        # Sort prize data in order of highest in the bin to lowest in the bin
        self._sort_json_data()

        # Update current prize being picked
        self.current_prize_id = 0
        self._update_current_prize_being_picked()

        # Create marker publisher
        self.marker_publisher_ = self.create_publisher(Marker, "visualization_marker", 2)     # 2 is the queue size
        self.get_logger().info("Created marker publisher...")

        # Create joint state subscriber
        self.state_subscriber_ = self.create_subscription(JointState, "joint_states", self.state_callback, 10)    # 10 is the queue size
        self.state_subscriber_  # prevent unused variable warning

        # Add marker to RVIZ to visualize prize in bin      
        self._add_markers_to_rviz()
        

    def _parse_json_data(self, json_path):

        for filename in os.listdir(path=json_path):
            if "step_data" in filename:
                # skipping files with step_data in the name leaves the user free to choose 
                # the name of the file which holds the prize input data.
                continue
            else:
                full_path = json_path + filename

                with open(full_path,"r") as f:
                    prize_data = json.load(f)
                    for prize in prize_data["prizes"]:
                      self.prize_data.append(prize)


    def _sort_json_data(self):
        '''
            Sorts prize data in order from highest in the bin to lowest in the bin.
        '''
        sorted_prize_data = []
        z_values = [prize["position"]["z"] for prize in self.prize_data]

        z_values.sort(reverse=True)

        for value in z_values:
            for i in range(len(self.prize_data)):
                if self.prize_data[i]["position"]["z"] == value:
                    sorted_prize_data.append(self.prize_data[i])

        self.prize_data = sorted_prize_data


    def _update_current_prize_being_picked(self):
        if self.current_prize_id < len(self.prize_data):
          current_prize = self.prize_data[self.current_prize_id]
          self.current_prize = [current_prize["position"]["x"], current_prize["position"]["y"], 
                                current_prize["position"]["z"], CLOSED_CLAW_DISTANCE]
        else:
            self.get_logger().info("All prizes picked...")


    def state_callback(self, joint_state: JointState):

        if np.allclose(self.current_prize, joint_state.position):
            # delete prize marker from RVIZ
            prize = Marker()
            prize.header.stamp = self.get_clock().now().to_msg()
            prize.header.frame_id = "bin"
            prize.id = self.current_prize_id
            prize.action = 2    # 2 -> deletes an object
            self.marker_publisher_.publish(prize)

            # Update current prize
            self.current_prize_id += 1
            self._update_current_prize_being_picked()
            

    def _add_markers_to_rviz(self):
        for id, prize in enumerate(self.prize_data):
            prize_marker = Marker()
            prize_marker.header.stamp = self.get_clock().now().to_msg()
            prize_marker.header.frame_id = "bin"
            prize_marker.id = id
            prize_marker.type = 1    # 1 -> cube
            prize_marker.action = 0  # 0 -> add/modify an object
            prize_marker.pose.position.x = prize["position"]["x"]
            prize_marker.pose.position.y = prize["position"]["y"]
            prize_marker.pose.position.z = prize["position"]["z"]
            prize_marker.scale.x = prize["bounding_box"]["x"]
            prize_marker.scale.y = prize["bounding_box"]["y"]
            prize_marker.scale.z = prize["bounding_box"]["z"]
            prize_marker.color.r = random.random()
            prize_marker.color.g = random.random()
            prize_marker.color.b = random.random()
            prize_marker.color.a = 1.0 

            self.marker_publisher_.publish(prize_marker)
            self.get_logger().info("published prize marker...")
            rclpy.spin_once(self)


def main(args=None):

    if args is None:
        args = sys.argv
    
    rclpy.init(args=args)

    json_file_path = args[1]

    visualize_prizes = VisualizePrizes(json_path=json_file_path)

    rclpy.spin(visualize_prizes)


if __name__ == '__main__':
    main()
