# ros2_challenge

## This challenge uses ROS2 to create a motion plan for a claw arm that picks up and drops off prizes from a bin.

### Setup Instructions
`git clone https://github.com/Anna-LeeMcLean/ros2_challenge.git`

`cd ros2_challenge`

`colcon build`

`source install/setup.bash`

This ROS2 workspace holds a package called 'motion_planner' which runs a node that accepts json data and a file path. The json data contains information about the location and sizes of the prizes to be picked from the bin. The node parses the json data, creates a motion plan for each prize and outputs the data for the motion plans to json files. 

Each output json file holds the states the claw arm must execute to pick up and drop-off one prize. It also holds which prize was picked and the total length of time for the motion. The output json files are dumped to the same file path as the input json file.

### Motion Planner Node Execution

`ros2 run motion_planner planner "<path/to/json/folder>" "<json_file_name.json>"
`

A json file called 'prize_data.json' is currently in the json folder to serve as an example. To run the node with the current json input from the top level folder:

`ros2 run motion_planner planner "./src/motion_planner/json/" "prize_data.json"
`

The output json files for each prize are called 'step_data_prize_[i].json' with i being the prize number.

### Assumptions
1. The order in which the prizes are picked is up to the programmer.
2. Time taken to execute a state is rounded up to the nearest integer.

### Visualization Node Execution

The workspace also holds packages which allows for visualization of the robot arm picking up and dropping off prizes. Launch instructions and a video is provided below.

`ros2 launch state_visualizer claw_arm_scene.launch.py`

In a separate terminal:

`ros2 launch state_visualizer state_publisher.launch.py`

