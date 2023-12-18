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

OPEN_CLAW_DISTANCE = 0.3                    # width between claw fingers. this is the travel distance needed to fully open the claw based on the URDF
CLOSED_CLAW_DISTANCE = 0.0
CLAW_ACTIVATION_TIME = 1.0                  # time taken to fully open/close claw in seconds
CLAW_SPEED_M_PER_S = 0.2                    # claw speed in m/s (NOTE: this is the speed of the links which move the claw and not the speed at which the claw opens/closes)
PUBLISH_RATE = 10                           # rate at which joint state updates are sent on the /joint_states topic
LINK_STEP_M = CLAW_SPEED_M_PER_S/PUBLISH_RATE    # interpolation distance needed to update joint positions so that translation occurs at joint speed
CLAW_STEP_M = OPEN_CLAW_DISTANCE/(CLAW_ACTIVATION_TIME * PUBLISH_RATE)    # interpolation distance needed to update claw position so that translation takes 1 second

class StatePublisher(Node):
    def __init__(self, json_path) -> None:
        super().__init__("state_publisher")

        # Get json data
        self.step_data = []
        self.prize_data = []

        # Open json data and populate the two attributes above
        self._get_prize_and_step_json_data(json_path=json_path)

        # Sort step data
        self._sort_json_data()

        self.get_logger().info(f"{self.step_data = }")

        # Define joint/link names from URDF and joint velocities
        self.joint_names = ["base_to_link1", "link1_to_link2", 
                      "link2_to_link3", "claw_leg1_to_claw_leg2"]
        
        # Create joint state publisher
        self.joint_state_commands = SimpleQueue()
        self.js_publisher_ = self.create_publisher(JointState, "update_joint_states", 10)     # 10 is the queue size
        self.timer = self.create_timer(1/PUBLISH_RATE, self.timer_callback)         # this timer enforces the publish rate
        self.get_logger().info("Created joint command publisher...")

        # Set claw arm initial position
        j_initial = self._make_joint_state()
        j_initial.position = [self.step_data[0]["states"][0]["position"]["x"],
                              self.step_data[0]["states"][0]["position"]["y"],
                              self.step_data[0]["states"][0]["position"]["z"],
                              CLOSED_CLAW_DISTANCE]
        self.js_publisher_.publish(j_initial)
        self.get_logger().info("Set initial claw arm position.")
        

    def _get_prize_and_step_json_data(self, json_path):

        for filename in os.listdir(path=json_path):
            if "step_data" in filename:
                full_path = json_path + filename

                with open(full_path,"r") as f:
                    self.step_data.append(json.load(f))
            else:
                full_path = json_path + filename

                with open(full_path,"r") as f:
                    prize_data = json.load(f)
                    for prize in prize_data["prizes"]:
                      self.prize_data.append(prize)

    def _sort_json_data(self):

        step_data_dict = {}

        for step_data in self.step_data:
            corresponding_prize = self.prize_data[step_data["prize_picked"]]
            prize_z = corresponding_prize["position"]["z"]
            step_data_dict.update({prize_z: step_data})

        step_data_sorted = dict(sorted(step_data_dict.items(), reverse=True))
        self.step_data = [step_data for step_data in step_data_sorted.values()]


    def timer_callback(self):

        if not self.joint_state_commands.empty():
            command = self.joint_state_commands.get()
            self.js_publisher_.publish(command)
        else:
            # Joint states are added to the queue and then the node spins. 
            # Therefore this queue should be fully populated before getting states from it.
            self.get_logger().info(f"All joint commands published.")
        

    def _create_and_add_joint_states_to_queue(self, current_state_data:dict, previous_state_data:dict) -> None:

        old_x = previous_state_data["position"]["x"]
        old_y = previous_state_data["position"]["y"]
        old_z = previous_state_data["position"]["z"]

        if previous_state_data["claw_state"]:
            old_claw_position = OPEN_CLAW_DISTANCE
        else:
            old_claw_position = CLOSED_CLAW_DISTANCE

        if current_state_data["claw_state"] == previous_state_data["claw_state"]:

            # Send update for x translation
            new_x = current_state_data["position"]["x"]
            delta_x = new_x - old_x
            if delta_x != 0:
                number_of_commands_for_x_travel= (abs(delta_x)/CLAW_SPEED_M_PER_S) * PUBLISH_RATE

                for _ in range(int(number_of_commands_for_x_travel)):
                    joint_state = self._make_joint_state()

                    if delta_x > 0:
                      joint_state.position = [old_x + LINK_STEP_M, old_y, old_z, old_claw_position]
                      old_x += LINK_STEP_M
                    else:
                      joint_state.position = [old_x - LINK_STEP_M, old_y, old_z, old_claw_position]
                      old_x -= LINK_STEP_M

                    self.joint_state_commands.put(joint_state)

            # Send update for y translation
            new_y = current_state_data["position"]["y"]
            delta_y = new_y - old_y
            if delta_y != 0:
                number_of_commands_for_y_travel= (abs(delta_y)/CLAW_SPEED_M_PER_S) * PUBLISH_RATE

                for _ in range(int(number_of_commands_for_y_travel)):
                    joint_state = self._make_joint_state()

                    if delta_y > 0:
                        joint_state.position = [old_x, old_y + LINK_STEP_M, old_z, old_claw_position]
                        old_y += LINK_STEP_M
                    else:
                        joint_state.position = [old_x, old_y - LINK_STEP_M, old_z, old_claw_position]
                        old_y -= LINK_STEP_M

                    self.joint_state_commands.put(joint_state)

            # Send update for z translation
            new_z = current_state_data["position"]["z"]
            delta_z = new_z - old_z
            if delta_z != 0:
                number_of_commands_for_z_travel= (abs(delta_z)/CLAW_SPEED_M_PER_S) * PUBLISH_RATE

                for _ in range(int(number_of_commands_for_z_travel)):
                    joint_state = self._make_joint_state()

                    if delta_z > 0:
                        joint_state.position = [old_x, old_y, old_z + LINK_STEP_M, old_claw_position]
                        old_z += LINK_STEP_M
                    else:
                        joint_state.position = [old_x, old_y, old_z - LINK_STEP_M, old_claw_position]
                        old_z -= LINK_STEP_M

                    self.joint_state_commands.put(joint_state)
                                
        elif (previous_state_data["claw_state"] == True) and (current_state_data["claw_state"] == False):
            
            self._create_claw_activation_joint_states(x=old_x, y=old_y, z=old_z, 
                                                      old_claw_pos=old_claw_position, 
                                                      activation_direction="close")

        elif (previous_state_data["claw_state"] == False) and (current_state_data["claw_state"] == True):
            
            self._create_claw_activation_joint_states(x=old_x, y=old_y, z=old_z, 
                                                      old_claw_pos=old_claw_position, 
                                                      activation_direction="open")

        
    def _create_claw_activation_joint_states(self, x: float, y:float, z: float, old_claw_pos: float, activation_direction: str):

        # Send update for activating claw
        number_of_commands_for_claw_activation = CLAW_ACTIVATION_TIME * PUBLISH_RATE

        for _ in range(int(number_of_commands_for_claw_activation)):
            joint_state = self._make_joint_state()

            if activation_direction == "open":
              joint_state.position = [x, y, z, old_claw_pos + CLAW_STEP_M]
              old_claw_pos += CLAW_STEP_M

              self.joint_state_commands.put(joint_state)

            elif activation_direction == "close":
              joint_state.position = [x, y, z, old_claw_pos - CLAW_STEP_M]
              old_claw_pos -= CLAW_STEP_M

              self.joint_state_commands.put(joint_state)
        

    def _make_joint_state(self) -> JointState:
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names

        return joint_state

    def create_joint_commands(self):
        
        for step_data in self.step_data:
        
            previous_state = step_data["states"][0]
            
            for state in step_data["states"]:
                self._create_and_add_joint_states_to_queue(current_state_data=state, previous_state_data=previous_state)
                previous_state = state


def main(args=None):

    if args is None:
        args = sys.argv
    
    rclpy.init(args=args)

    json_file_path = args[1]

    state_publisher = StatePublisher(json_path=json_file_path)
    state_publisher.create_joint_commands()

    rclpy.spin(state_publisher)

    state_publisher.get_logger().info(f"finished publishing states. destroying publisher node...")
    state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
