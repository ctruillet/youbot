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
import time
import copy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from math import inf, sqrt, cos, sin

print_elt ="""
<?xml version="1.0"?>
<robot name="cube">
  <gazebo>
    <static>true</static>
  </gazebo>
  <link name="box">
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.01 0.01 0.01"/>
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
        self.controlPoints = []
        self.lastPosition = [inf, inf]

        self.fileControlPoint = open("./src/youbot_print/resource/controlPoints.csv","w")
        self.fileControlPoint.write("x, y\n")

    def listener_callback(self, msg):
        x = round(msg.pose.pose.position.x, 3)
        y = round(msg.pose.pose.position.y, 3)
        
        if( self.lastPosition[0] - x > 0.001 or
            self.lastPosition[1] - y > 0.001):

            self.lastPosition = [x ,y]
            
            self.saveControlPoints((x,y))
            self.printing((x, y))
        else:
            self.lastPosition = [x ,y]

        

    def printing(self, P):
        x = copy.copy(P[0])
        y = copy.copy(P[1])
        self.get_logger().info("Print a cube a the position : {x:%f, y:%f}" % (x,y))
        msg = "ros2 run gazebo_ros spawn_entity.py -entity cube" + str(x) + "" + str(y) + " -x " + str(x) + " -y " + str(y) + " -file $PWD/src/youbot_print/resource/cube.urdf"
        returned_value = subprocess.call(msg, shell=True)  # returns the exit code in unix

    def saveControlPoints(self, P):
        self.fileControlPoint.write(str(P[0]) + "," + str(P[1]) + "\n")

    # def printing(self, A, B):
    #     self.printCurently = True
    #     xA = A[0]
    #     yA = A[1]
    #     xB = B[0]
    #     yB = B[1]
    #     distance = sqrt((xB-xA)**2 + (yB-yA)**2)
    #     if (distance > 10 or distance <= 0.01):
    #         self.printCurently = False
    #         return 

    #     self.get_logger().info(" >>> Distance : %f" % distance)

    #     t = 0
    #     while (t <= distance):
    #         x = xA + t * (xB-xA)
    #         y = yB + t * (yB-yA)

    #         self.get_logger().info("Print a cube a the position : {x:%f, y:%f}" % (x,y))
    #         msg = "ros2 run gazebo_ros spawn_entity.py -entity cube" + str(x) + "" + str(y) + " -x " + str(x) + " -y " + str(y) + " -file $PWD/src/youbot_print/resource/cube.urdf"
    #         returned_value = subprocess.call(msg, shell=True)  # returns the exit code in unix

    #         t += 0.02

    #     self.printCurently = False


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
