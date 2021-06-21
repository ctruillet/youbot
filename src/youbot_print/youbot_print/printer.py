# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import subprocess
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from math import inf

cube_urdf = """ 
<?xml version="1.0"?>
<robot name="cube">
  <gazebo>
    <static>true</static>
  </gazebo>
  <link name="box">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
"""

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('youbot_print')
        self.get_logger().info("subscribe")
        self.subscription = self.create_subscription(
            Odometry,
            '/youbot/p3d',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.lastPosition = [inf, inf, inf]

    def listener_callback(self, msg):
        x = round(msg.pose.pose.position.x, 3)
        y = round(msg.pose.pose.position.y, 3)
        z = round(msg.pose.pose.position.z, 3)
        
        if( self.lastPosition[0] - x > 0.001 or
            self.lastPosition[1] - y > 0.001 or
            self.lastPosition[2] - z > 0.001):
            self.printing(x, y, z)

        self.lastPosition = [x ,y ,z]

    def printing(self, x, y, z):
        self.get_logger().info("Print a cube a the position : {x:%f, y:%f, z:%f}" % (x,y,z))
        msg = "ros2 run gazebo_ros spawn_entity.py -entity " + str(x) + "" + str(y) + "" + str(z) + " -x " + str(x) + " -y " + str(y) + " -file $PWD/src/youbot_print/resource/cube.urdf"
        
        returned_value = subprocess.call(msg, shell=True)  # returns the exit code in unix



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
