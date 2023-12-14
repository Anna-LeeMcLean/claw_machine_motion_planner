from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
    
    json_path = os.path.join(get_package_share_directory('motion_planner'),"json/")

    return LaunchDescription([
        Node(
        package="state_visualizer",
        executable="joint_command_publisher",
        arguments=[json_path]
        ),
        Node(
        package="state_visualizer",
        executable="prize_marker_publisher",
        output="screen",
        arguments=[json_path],
        )
    ])
