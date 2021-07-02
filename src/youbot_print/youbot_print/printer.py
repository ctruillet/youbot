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
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs import Int32

from math import inf, sqrt, cos, sin, atan2

xOffset = 0.484
yOffset = -0.042
zOffset = 0.0


class Printer(Node):
	def __init__(self):
		super().__init__('youbot_print')
		self.get_logger().info("subscribe")
		self.subscription = self.create_subscription(
			Odometry,
			'/youbot/p3d',
			self.listener_callback_P3D,
			10)
		self.subscription = self.create_subscription(
			Int32,
			'/youbot/cmd_level',
			self.listener_callback_CMD_LEVEL,
			10)
		self.subscription  # prevent unused variable warning
		self.printCurently = False
		self.controlPoints = []
		self.zlevel = 0.0
		self.lastPosition = np.array([0.0, 0.0, 0.0])

	def listener_callback_P3D(self, msg):
		P = np.array([round(msg.pose.pose.position.x, 3),round(msg.pose.pose.position.y, 3),round(msg.pose.pose.position.z, 3)])
		
		if (np.linalg.norm(P - self.lastPosition) > 0.05):
			lastP = copy.copy(self.lastPosition)
			self.lastPosition = copy.copy(P)
			self.get_logger().info("================================================================================")
			self.get_logger().info("GET POINT")
			self.get_logger().info("\t {P:{x:%.3f, y:%.3f, z:%.3f}}" % (P[0], P[1], P[2]) )
			self.get_logger().info("================================================================================")
			self.waitUntil(self.printCurently)
			self.printing(lastP, P)

	def listener_callback_CMD_LEVEL(self, msg):
		self.zlevel = msg


    # if( self.lastPosition[0] - x > 0.001 or
    #     self.lastPosition[1] - y > 0.001):
    #   P = (copy.copy(self.lastPosition[0]),copy.copy(self.lastPosition[1]))
    #   self.lastPosition = [x ,y]
    #   self.waitUntil(self.printCurently)
    #   self.printing(P, (x, y))
      
    # self.lastPosition = [x ,y]
        
	def printing(self, A, B):
		self.printCurently = True
		xA = A[0]
		yA = A[1]
		zA = A[2]
		xB = B[0]
		yB = B[1]
		zB = B[2]

		distance = np.linalg.norm(B - A)
		if (distance <= 0.05 or distance >= 100):
			self.printCurently = False
			return 

		angle = atan2(yB - yA, xB - xA)

		#self.get_logger().info(" >>> Distance : %f" % distance)
		self.get_logger().info("================================================================================")
		self.get_logger().info("PRINTING")
		self.get_logger().info("\t {A:{x:%.3f, y:%.3f, z:%.3f}}" % (xA, yA, zA) )
		self.get_logger().info("\t {B:{x:%.3f, y:%.3f, z:%.3f}}" % (xB, yB, zB))
		self.get_logger().info("\t {angle:%.3f}}" % (angle))
		self.get_logger().info("================================================================================")
		msg = "ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' '{name: \"cube"
		msg = msg + str(xA) + "" + str(yA) + "\", xml: \"<?xml version=\\\"1.0\\\"?> <sdf version=\\\"1.5\\\"><model name=\\\"cube\\\"><static>true</static><link name=\\\"box\\\"><pose frame=''>"
		msg = msg + str((xA+xB)/2 + xOffset) + " " + str((yA+yB)/2 + yOffset) + " " + str((zA+zB)/2 + zOffset + self.zlevel*0.2) + " 0.0 0.0 " + str(angle) 
		msg = msg + "</pose><inertial><mass>0.01</mass><inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia></inertial>"
		msg = msg + "<visual name='box_visual'><geometry><box><size>"
		msg = msg + str(distance) + " 0.01 0.01"
		msg = msg + "</size></box></geometry></visual><self_collide>0</self_collide><kinematic>0</kinematic><gravity>1</gravity></link></model></sdf>"
		msg = msg + "\"}'"

		# self.get_logger().info(" >>> MSG : %s" % msg)

		returned_value = subprocess.call(msg, shell=True)  # returns the exit code in unix

		self.printCurently = False

	def waitUntil(self, boolean):
		while boolean:
			time.sleep(0.1)
		pass

def main(args=None):
	rclpy.init(args=args)

	minimal_subscriber = Printer()

	rclpy.spin(minimal_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
