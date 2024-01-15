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
            [0.5424163089601358, 1.795103312222939, 0.7703851927883497, -0.6375787439465476],
            [0.6193814176940633, 1.8403627316701683,
                0.6951174561166101, -0.71889618318779],
            [0.54326491823905945, -0.08492943703618432,
                0.05142371183959614, -0.9986769256674743]]

        self.pose_2 = [
            [0.8496426664358298, 1.0643705082357566, 0.6418564386743822, 0.7668248249321608],
            [0.6651600994226751, 1.7372557560558928,
                0.7046241454729849, 0.709580730866098],
            [0.07282529682623694, 4.244794768906096,
                0.4050570703577275, 0.9142914030839484],
            [-1.6290334007714011, 4.1654602694088, 0.7662243361654592,-0.6425731605566805]]

        self.pose_3 = [

            [0.591973105402961,1.0400530648816118, 0.6538282408565073,
                0.7566430013278952],
            [0.20788959769330756, 2.6194065623529768,0.006984574790139538,-0.9999756075600049]]

        # Set our demo's initial pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = -0.17823025769959178
        self.initial_pose.pose.position.y = 0.022046848424018568
        self.initial_pose.pose.orientation.z = -0.999522981849674 
        self.initial_pose.pose.orientation.w = 0.030883794364298032
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
        self.navStart3 = True
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
                self.route_poses_3 = []
                self.pose = PoseStamped()
                self.pose.header.frame_id = 'map'
                self.pose.header.stamp = self.navigator.get_clock().now().to_msg()
                self.nextTargetPose = Int32()

                # try:
                if self.goalCmd == 1 and self.navStart1 == True:  # Command From Labview to goto Pose 1
                    print("going to Pose 1")
                    self.tracerStatus.data = 4  # Ongoing
                    self.arucoMove = True
                    self.next_pose = 2
                    # self.goalStatus_pub.publish(self.tracerStatus)
                    self.navStart1 = False
                    movmnt_start = self.navigator.get_clock().now()
                    movm_time = self.navigator.get_clock().now()
                    while (movm_time-movmnt_start) < Duration(seconds=4.0):
                        self.tracer_cmd.linear.x = -0.20
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

                if self.goalCmd == 2 and self.navStart2 == True:
                    self.tracerStatus.data = 4  # Ongoing status
                    self.arucoMove = True
                    self.next_pose = 1
                    self.navStart2 = False
                    # self.nextTargetPose.data = 1
                    print("going to 2")
                    movmnt_start = self.navigator.get_clock().now()
                    movm_time = self.navigator.get_clock().now()
                    while (movm_time-movmnt_start) < Duration(seconds=3.0):
                        self.tracer_cmd.linear.x = -0.20
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

                if self.goalCmd == 3 and self.navStart3 == True:
                    self.tracerStatus.data = 4  # Ongoing status
                    self.arucoMove = True
                    self.navStart3 = False
                    print("going to 3")
                    movmnt_start = self.navigator.get_clock().now()
                    movm_time = self.navigator.get_clock().now()
                    while (movm_time-movmnt_start) < Duration(seconds=4.0):
                        self.tracer_cmd.linear.x = -0.20
                        self.cmd_pub.publish(self.tracer_cmd)
                        movm_time = self.navigator.get_clock().now()

                    for pt in self.pose_3:
                        self.pose.pose.position.x = pt[0]
                        self.pose.pose.position.y = pt[1]
                        self.pose.pose.orientation.w = pt[2]
                        self.pose.pose.orientation.z = pt[3]
                        self.route_poses_3.append(deepcopy(self.pose))
                    self.navigator.goThroughPoses(self.route_poses_3)
                    #

                if self.navigator.isTaskComplete() and (self.navStart1 == False or self.navStart2 == False or self.navStart3==False):
                    if self.goalCmd == 1:

                        if self.arucoFlag == True:
                            if self.arucoDist > 0.30 and self.arucoMove == True:
                                self.tracer_cmd.linear.x = +0.10
                                # if self.arucoOrnt < 0.0:
                                #     self.tracer_cmd.angular.z = +0.10
                                # if self.arucoOrnt > 0.05:
                                #     self.tracer_cmd.angular.z = -0.10
                                self.cmd_pub.publish(self.tracer_cmd)
                            elif self.arucoDist < 0.30:
                                print("Pose 1 Reached")
                                self.arucoMove = False
                                self.tracer_cmd.linear.x = 0.0
                                self.tracer_cmd.angular.z = 0.0
                                self.cmd_pub.publish(self.tracer_cmd)
                                self.arucoFlag = False

                                if self.goalCmd == 1:
                                    self.tracerStatus.data = 1 # Goal 1 Reached status
                                    self.navStart2 = True
                                    self.navStart3 = True

                    if self.goalCmd == 2:
                            if self.arucoFlag == True:
                                if self.arucoDist > 0.50 and self.arucoMove == True:
                                    self.tracer_cmd.linear.x = +0.10
                                    if self.arucoOrnt < 0.1:
                                        self.tracer_cmd.angular.z = +0.10
                                    if self.arucoOrnt > 0.0:
                                        self.tracer_cmd.angular.z = -0.10
                                    self.cmd_pub.publish(self.tracer_cmd)
                                elif self.arucoDist < 0.50:
                                    print("Pose 2 Reached")
                                    self.arucoMove = False
                                    self.tracer_cmd.linear.x = 0.0
                                    self.tracer_cmd.angular.z = 0.0
                                    self.cmd_pub.publish(self.tracer_cmd)
                                    # Publishing reached status
                                    self.arucoFlag = False
                                    if self.goalCmd == 2:
                                        self.tracerStatus.data = 2 # Goal 2 Reached status
                                        self.navStart1 = True

                    if self.goalCmd == 3:
                            if self.arucoFlag == True:
                                if self.arucoDist > 0.20 and self.arucoMove == True:
                                    self.tracer_cmd.linear.x = +0.10
                                    if self.arucoOrnt < 0.1:
                                        self.tracer_cmd.angular.z = +0.10
                                    if self.arucoOrnt > 0.0:
                                        self.tracer_cmd.angular.z = -0.10
                                    self.cmd_pub.publish(self.tracer_cmd)
                                elif self.arucoDist < 0.20:
                                    print("Pose 3 Reached")
                                    self.arucoMove = False
                                    self.tracer_cmd.linear.x = 0.0
                                    self.tracer_cmd.angular.z = 0.0
                                    self.cmd_pub.publish(self.tracer_cmd)
                                    self.arucoFlag = False
                                    if self.goalCmd == 3:
                                        self.tracerStatus.data = 3 # Goal 3 Reached status
                                        self.navStart1 = True

            self.goalStatus_pub.publish(self.tracerStatus)

            if (self.startCmd == 0 and self.navStart1 == False) or (self.startCmd == 0 and self.navStart2 == False) or (self.startCmd == 0 and self.navStart3 == False):
                print("Cancelling the goal")
                self.navigator.cancelTask()
                result = self.navigator.getResult()
                print('result', result)
                self.navStart1 = True
                self.navStart2 = True
                self.navStart3 = True
                print("Cancelled the goal")
                self.tracerStatus.data = 5 #Emergency Stop status
            result = self.navigator.getResult()
        except:
            pass


def main():
    rclpy.init()
    poseNav = goalNavigation()

    rclpy.spin(poseNav)


if __name__ == '__main__':
    main()
