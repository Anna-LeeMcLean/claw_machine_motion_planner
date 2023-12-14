from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os.path
import xacro

def generate_launch_description():

    rviz_config_file = PathJoinSubstitution([FindPackageShare("claw_arm_description"), "rviz_config", "claw_arm.rviz"])
    
    urdf_file_name = 'claw_arm.urdf.xacro'
    urdf_file = os.path.join(get_package_share_directory("claw_arm_description"), "urdf", urdf_file_name)

    urdf_xacro_parse = xacro.parse(open(urdf_file))
    xacro.process_doc(urdf_xacro_parse)

    robot_description = {"robot_description": urdf_xacro_parse.toxml()}
    
    json_path = os.path.join(get_package_share_directory('motion_planner'),"json/")
    json_step_data_file_name = "step_data_prize_0.json"

    jsp_params = {"source_list": ["update_joint_states"], "rate": 100}


    return LaunchDescription([
        Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
        ),
        Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[jsp_params],
        ),
        Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        )
    ])

'''
Node(
        package="state_visualizer",
        executable="joint_command_publisher",
        arguments=[json_path, json_step_data_file_name]
        ),
'''
