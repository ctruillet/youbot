# Copyright (c) 2020 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This launch file shows how to launch robot_state_publisher with a simple
# URDF read from a file (found using ament_get_package_share_directory) and
# passed as the 'robot_description' parameter.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

import launch
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():

    urdf_dir = os.path.join(get_package_share_directory('youbot_description'), 'urdf')
    urdf_file = os.path.join(urdf_dir, 'youbot.urdf')

    xml = open(urdf_file, 'r').read()
    xml = xml.replace('"', '\\"')

    world = os.path.join(get_package_share_directory('youbot_description'), 'sdf', 'world.sdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return launch.LaunchDescription([
        Node(package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_desc}]),

        # Node(
        #     name='joint_state_publisher_gui',
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     remappings=[
        #         ('/joint_states','/youbot/set_joint_trajectory')
        #     ],
        #     parameters=[{'use_gui': True}],
        # ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', LaunchConfiguration('use_sim_time', default='true')],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', '{name: \"youbot\", xml: \"'  +  xml + '\" }'],
            output='screen'),

        Node(
            package='teleop_twist_keyboard',
            namespace='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            remappings=[
                ('/teleop_twist_keyboard/cmd_vel','/youbot/cmd_vel')
            ],
            output='screen',
            prefix = 'xterm -e'
        )
    ])