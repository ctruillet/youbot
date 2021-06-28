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

from math import inf, sqrt, cos, sin, atan2


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
    self.printCurently = False
    self.controlPoints = []
    self.lastPosition = [inf, inf]

  def listener_callback(self, msg):
    x = round(msg.pose.pose.position.x, 3)
    y = round(msg.pose.pose.position.y, 3)
        
    if( self.lastPosition[0] == inf or
        self.lastPosition[1] == inf):   
        self.lastPosition = [x ,y]

    if( self.lastPosition[0] - x > 0.001 or
        self.lastPosition[1] - y > 0.001):
      P = (copy.copy(self.lastPosition[0]),copy.copy(self.lastPosition[1]))
      self.lastPosition = [x ,y]
      self.waitUntil(self.printCurently)
      self.printing(P, (x, y))
      
    self.lastPosition = [x ,y]
        
  def printing(self, A, B):
    self.printCurently = True
    xA = A[0]
    yA = A[1]
    xB = B[0]
    yB = B[1]
    distance = sqrt((xB-xA)**2 + (yB-yA)**2)
    if (distance <= 0.01 or distance > 100):
      self.printCurently = False
      return 

    angle = atan2(yB - yA, xB - xA)

    self.get_logger().info(" >>> Distance : %f" % distance)

    self.get_logger().info("Print a cube a the position : {x:%f, y:%f, angle:%f}" % (xA,yA,angle))
    msg = "ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' '{name: \"cube"
    msg = msg + str(xA) + "" + str(yA) + "\", xml: \"<?xml version=\\\"1.0\\\"?> <sdf version=\\\"1.5\\\"><model name=\\\"cube\\\"><static>true</static><link name=\\\"box\\\"><pose frame=''>"
    msg = msg + str(xA) + " " + str(yA) + " " +"0.01 0.0 0.0" + str(angle) 
    msg = msg + "</pose><inertial><mass>0.01</mass><inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia></inertial>"
    msg = msg + "<visual name='box_visual'><geometry><box><size>"
    msg = msg + str(round(distance,3)) + " 0.01 0.01"
    #msg = msg + str(max(round(abs(yB-yA),3),0.01)) + " " + str(max(round(abs(xB-xA),3),0.01)) + " 0.01"
    msg = msg + "</size></box></geometry></visual><self_collide>0</self_collide><kinematic>0</kinematic><gravity>1</gravity></link></model></sdf>"
    msg = msg + "\"}'"

    # self.get_logger().info(" >>> MSG : %s" % msg)

    returned_value = subprocess.call(msg, shell=True)  # returns the exit code in unix

    self.printCurently = False

  def waitUntil(self, boolean):
    while boolean:
      time.sleep(0.1)

        

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
