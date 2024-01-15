#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from copy import deepcopy
from sre_constants import SUCCESS

from geometry_msgs.msg import PoseStamped, Twist, PoseArray, Point
from goal_navigation.robot_navigation import BasicNavigator, TaskResult
from std_msgs.msg import Bool, Int32
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from goal_interfaces.msg import Navgoal
import message_filters
from message_filters import TimeSynchronizer, Subscriber
# from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
import time
"""
Basic security route patrol demo. In this demonstration, the expectation
is that there are security cameras mounted on the robots recording or being
watched live by security staff.
"""


class goalNavigation(Node):

    def __init__(self):
        super().__init__('poseNav')

        self.navigator = BasicNavigator()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tracer_cmd = Twist()
        self.goalStatus_pub = self.create_publisher(Int32, '/tracerStatus', 10)
        # self.goalCmd_pub = self.create_publisher(Int32, '/labviewint', 10)
        # Security route, probably read in from a file for a real application
        # from either a map or drive and repeat.
        self.pose_1 = [
            # [-0.04923155631271733, 6.429067689657896, -0.65147, 0.7586],
            [1.636586182770103, 5.733010893697215,
                -0.3926401658626039, 0.9196921768458112],
            [2.0853580475789166, 3.497341558047864,
                -0.7028218811246014, 0.7113658716950628],
            [2.114444677172994, 2.0613619815467588, 0.07222120230898153, -0.997388639366343]]

        self.pose_2 = [
            [2.1839935021348382, 2.082335910255417, -1.0, 0.0],
            [1.7092743308154927, 6.5968739818615845,
                0.7238265429552857, 0.6899819821657663],
            [1.3591987613561451, 6.5968739818615845,
                0.7238265429552857, 0.6899819821657663],
            [0.1467747652138115, 6.2695717507714575, -0.6449691616571311,0.7642085975120257]]

        # Set our demo's initial pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 1.4361418692092276
        self.initial_pose.pose.position.y = 2.0191494318999808
        self.initial_pose.pose.orientation.z = -0.9998116213141753
        self.initial_pose.pose.orientation.w = 0.0194093246951081
        self.navigator.setInitialPose(self.initial_pose)
        local_costmap = self.navigator.getLocalCostmap()

        # self.startSub = Subscriber('startCmd', Navgoal)
        # self.stopSub = Subscriber('stopCmd', Navgoal)
        # ts = TimeSynchronizer(
        #     [self.startSub, self.stopSub], 10)
        # ts.registerCallback(self.goalCallback)
        # rclpy.spin()
        # self.stopCmd = None
        # self.stopsub = self.create_subscription(
        #     Navgoal,
        #     'stopCmd',
        #     self.stopCallback,
        #     1)

        self.navStart2 = True
        self.navStart1 = True
        self.arucoFlag = False
        self.arucoMove = True
        self.tracerStatus = Int32()
        # self.tracerStatus.data = 0
# Subscriber List

        self.aruco_sub = self.create_subscription(PoseArray,
                                                  '/aruco_poses',
                                                  self.aruco_detect_callback,
                                                  1)

# Goal Position Command
        self.goalSub = self.create_subscription(
            Int32,
            '/station_pub',
            self.cmdCallback,
            1)

    # START /STOP Button
        self.navsub = self.create_subscription(
            Bool,
            '/lvbool',
            self.goalCallback,
            1)
        # self.navsub = self.create_subscription(
        #     'roscluster',
        #     'PoseMsg',
        #     self.poseCallback,
        #     1)

    def cmdCallback(self, goalcmd):  # Goal Position Command
        self.goalCmd = goalcmd.data

    def goalCallback(self, cmd):  # START /STOP Button
        self.startCmd = cmd.data
        self.navigationtoPose()

    def poseCallback(self, poseData):
        self.poseData = poseData
        self.currentPose.poseX = self.poseData.pose.position.x
        self.currentPose.poseY = self.poseData.pose.position.x
        self.currentPose.poseZ = self.poseData.pose.orientation.z
        self.currentPose.poseW = self.poseData.pose.orientation.w

    def aruco_detect_callback(self, aruco_data):
        # transforms = FiducialTransformArray()

        if aruco_data is not None:
            self.arucoPose = aruco_data.poses[0].position
            self.arucoOrientation = aruco_data.poses[0].orientation
            self.arucoDist = round(
                self.arucoPose.z, 2)
            self.arucoOrnt = round(
                self.arucoPose.x, 2)
            self.arucoFlag = True
        # except:
        #     print("no data")
        #     self.arucoFlag = False
        #     self.arucoOrnt = 0
        #     pass

    def arucoMove(self):
        while self.arucoDist < 0.5:
            print(self.arucoDist)

    def navigationtoPose(self):
        try:

            if self.startCmd == 1:

                self.route_poses_1 = []
                self.route_poses_2 = []
                self.pose = PoseStamped()
                self.pose.header.frame_id = 'map'
                self.pose.header.stamp = self.navigator.get_clock().now().to_msg()
                self.nextTargetPose = Int32()

                # try:
                if self.goalCmd == 1 and self.navStart1 == True:  # Command From Labview to goto Pose 1
                    print("going to Pose 1")
                    self.tracerStatus.data = 3  # Ongoing
                    self.arucoMove = True
                    self.next_pose = 2
                    # self.goalStatus_pub.publish(self.tracerStatus)
                    self.navStart1 = False
                    movmnt_start = self.navigator.get_clock().now()
                    movm_time = self.navigator.get_clock().now()
                    while (movm_time-movmnt_start) < Duration(seconds=3.0):
                        self.tracer_cmd.linear.x = -0.21
                        self.cmd_pub.publish(self.tracer_cmd)
                        movm_time = self.navigator.get_clock().now()

                    for pt in self.pose_1:
                        self.pose.pose.position.x = pt[0]
                        self.pose.pose.position.y = pt[1]
                        self.pose.pose.orientation.w = pt[2]
                        self.pose.pose.orientation.z = pt[3]
                        self.route_poses_1.append(deepcopy(self.pose))
                    self.navigator.goThroughPoses(self.route_poses_1)
                    self.nextTargetPose.data = 2
                    # self.tracerStatus.data = 1  # Not publishing now wait to finish

                if self.goalCmd == 2 and self.navStart2 == True:
                    self.tracerStatus.data = 3  # Ongoing status
                    self.arucoMove = True
                    self.next_pose = 1
                    # self.goalCmd_pub.publish(self.tracerStatus)
                    self.navStart2 = False
                    # self.nextTargetPose.data = 1
                    print("going to 2")
                    movmnt_start = self.navigator.get_clock().now()
                    movm_time = self.navigator.get_clock().now()
                    while (movm_time-movmnt_start) < Duration(seconds=3.0):
                        self.tracer_cmd.linear.x = -0.21
                        self.cmd_pub.publish(self.tracer_cmd)
                        movm_time = self.navigator.get_clock().now()

                    for pt in self.pose_2:
                        self.pose.pose.position.x = pt[0]
                        self.pose.pose.position.y = pt[1]
                        self.pose.pose.orientation.w = pt[2]
                        self.pose.pose.orientation.z = pt[3]
                        self.route_poses_2.append(deepcopy(self.pose))
                    self.navigator.goThroughPoses(self.route_poses_2)
                    #

                if self.navigator.isTaskComplete() and (self.navStart1 == False or self.navStart2 == False):
                    print("task completed")
                    
                    if self.arucoFlag == True:
                        if self.arucoDist > 0.45 and self.arucoMove == True:
                            self.tracer_cmd.linear.x = +0.10
                            # self.cmd_pub.publish(self.tracer_cmd)
                            if self.arucoOrnt < 0.0:
                                self.tracer_cmd.angular.z = +0.10
                            if self.arucoOrnt > 0.0:
                                self.tracer_cmd.angular.z = -0.10
                            self.cmd_pub.publish(self.tracer_cmd)
                        elif self.arucoDist < 0.45:
                            print("Pose Reached")
                            self.arucoMove = False
                            self.tracer_cmd.linear.x = 0.0
                            self.tracer_cmd.angular.z = 0.0
                            self.cmd_pub.publish(self.tracer_cmd)
                            # Publishing reached status
                            # self.goalCmd_pub.publish(self.tracerStatus)
                            # self.navStart = True
                            self.arucoFlag = False

                            if self.goalCmd == 1:
                                self.tracerStatus.data = 1
                                self.navStart2 = True
                                # time.sleep(60)
                                # self.goalCmd_pub.publish(self.nextTargetPose)

                            if self.goalCmd == 2:
                                self.tracerStatus.data = 2
                                # time.sleep(60)
                                self.navStart1 = True
                                # self.goalCmd_pub.publish(self.nextTargetPose)
                            # time.sleep(15)
            self.goalStatus_pub.publish(self.tracerStatus)

            if (self.startCmd == 0 and self.navStart1 == False) or (self.startCmd == 0 and self.navStart2 == False):
                print("Cancelling the goal")
                self.navigator.cancelTask()
                result = self.navigator.getResult()
                print('result', result)
                self.navStart1 = True
                self.navStart2 = True
                print("Canceled the goal")
                self.tracerStatus.data = 4
            result = self.navigator.getResult()
        except:
            pass


def main():
    rclpy.init()
    global stopCommand
    stopCommand = 1
    poseNav = goalNavigation()

    rclpy.spin(poseNav)

    # rclpy.shutdown()


if __name__ == '__main__':
    main()
