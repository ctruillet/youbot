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


class ControlArm(Node):

    def __init__(self):
        super().__init__('youbot_fake_moveit')
        self.get_logger().info("subscribe")

        self.publisherJointTrajectory_ = self.create_publisher(JointTrajectory, '/youbot/set_joint_trajectory', 10)
        self.timer = self.create_timer(0.5, self.setArmPosition)


    def setArmPosition(self):
        msg = JointTrajectory()
        msg.header.frame_id="world";
        msg.joint_names = ['youbot::arm_joint_1', 'youbot::arm_joint_2', 'youbot::arm_joint_3', 'youbot::arm_joint_4', 'youbot::arm_joint_5']
        msg.points = [JointTrajectoryPoint(positions=[3.1, 3.0, -1.0, 0.3, 2.9])]

        self.publisherJointTrajectory_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ControlArm()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
