import os
import sys
import rclpy

from ament_index_python.packages import get_package_share_directory 
from gazebo_msgs.srv import SpawnEntity

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    sdf_file_path  = os.path.join(get_package_share_directory("youbot_description"), "sdf", "Youbot", "model.sdf")
    # urdf_dir = os.path.join(get_package_share_directory('youbot_description'), 'urdf')
    # urdf_file = os.path.join(urdf_dir, 'youbot.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    return LaunchDescription([
        ExecuteProcess(
        #    cmd=['gazebo', '--verbose', 'sdf/world.sdf', '-s libgazebo_ros_factory.so'],
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', os.path.join(get_package_share_directory("youbot_description"), "sdf", "world.sdf")],
            output='screen'
        ),
        # Node(package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     output='both',
        #     parameters=[{'robot_description': robot_desc}]),
        # Node(
        #     package='teleop_twist_keyboard',
        #     namespace='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     remappings=[
        #         ('/teleop_twist_keyboard/cmd_vel','/youbot/cmd_vel')
        #     ],
        #     output='screen',
        #     prefix = 'xterm -e'
        # )
    ])

