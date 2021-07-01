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
import csv
import copy
import numpy as np
import time
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pi, cos, sin

windRose = {
    "W" : pi,
    "NW" : 3*pi/4.0,
    "N" : pi/2.0,
    "NE" : pi/4.0,
    "E" : 0.0,
    "SE" : -pi/4.0,
    "S" : -pi/2.0,
    "SW" : -3*pi/4.0,
}


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('youbot_fake_moveit')
        self.controlPointFile = "./src/youbot_fake_moveit/resource/controlPoints2.csv"
        self.position = np.array([0.0,0.0,0.0])
        self.angle = 0.0
        self.endMove = True

        self.publisherControlBaseTrajectory = self.create_publisher(Twist, '/youbot/cmd_vel', 10)

        self.controlList = []
        self.readFile()

        self.move(self.controlList)

        # self.get_logger().info("subscribe")

        # self.publisherJointTrajectory_ = self.create_publisher(JointTrajectory, '/youbot/set_joint_trajectory', 10)
        # self.timer = self.create_timer(0.5, self.setArmPosition)

    def readFile(self):
        with open(self.controlPointFile, mode='r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    #print(f'Column names are {", ".join(row)}')
                    line_count += 1

                direction = row["direction"]
                distance = row["distance"]
                level = row["level"]
                print(f' | {direction}\t{distance}\t{level}')

                self.genCommand(row["direction"], float(row["distance"]), int(row["level"]))
                
                line_count += 1

    def genCommand(self, direction, distance, level):
        angle = windRose.get(direction,self.angle)
        mvt = np.array([round(cos(angle) * distance,2),
                        round(sin(angle) * distance,2),
                        level - self.position[2]])


        # LEVEL
        if(abs(mvt[2]) > 0):
            msg = Twist()
            msg.linear.z = mvt[2]

        # ANGLE
        for i in range (int( round(16*abs(angle)/pi,0))):
            msg = Twist()
            msg.angular.z = angle/abs(angle) * pi/16.0
            #self.controlList.append(msg)

        if distance <= 0.0:
            pass

        # MOUVEMENT
        # print(f'   >>> MOVEMENT OF {mvt[0]}, {mvt[1]}, {mvt[2]}')
        for i in range (10*int(round(distance/0.1,0))):
            msg = Twist()
            if (mvt[0] != 0):
                msg.linear.x =  0.1
            else: 
                msg.linear.x =  0.0

            if (mvt[1] != 0):
                msg.linear.y =  0.1
            else: 
                msg.linear.y =  0.0   
            msg.angular.z = 0.0

            self.controlList.append(msg)

        # print("\t\tM", int(round(distance/0.1,0)), "A",int( round(16*abs(angle)/pi,0)))
                   

    def move(self, controlList):
        print(" BEGINNING OF THE MOVEMENT ")
        for msg in controlList:
            if(msg.linear.z != 0):
                self.changeLevel(msg.linear.z)
                pass
            else:
                self.publisherControlBaseTrajectory.publish(msg)
            time.sleep(0.2)

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.z = 0.0
        self.publisherControlBaseTrajectory.publish(msg)

        print(" END OF THE MOVEMENT ")
        

    def changeLevel(self, level):
        #ToDo
        pass



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