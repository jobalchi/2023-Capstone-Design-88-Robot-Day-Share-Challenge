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

import time

from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.node import Node

class holo_command(Node):
    def __init__(self):
        super().__init__('holo_command')

        self.pm_go = 0
        self.yolo_go = 0
        self.k_goal = 0
        self.val_kick_pose = None
        self.val_k_point = None

        # navigator
        self.navigator = BasicNavigator()

        # holo command publisher
        self.find_kick_pub = self.create_publisher(Int32, 'find_kick', 10)

        self.k_go_pub = self.create_publisher(Int32, 'k_go', 10)

        self.d_go_pub = self.create_publisher(Int32, 'd_go', 10)

        # holo command subscriber
        self.holo_nav_sub = self.create_subscription(Bool, 'holo_nav', self.nav_callback, 10)
        self.holo_nav_sub  # prevent unused variable warning

        self.kick_pose_sub = self.create_subscription(PoseStamped, 'kick_pose', self.kick_pose_callback, 10)
        self.kick_pose_sub  # prevent unused variable warning

        self.yolo_go_sub = self.create_subscription(Int32, 'yolo_go', self.yolo_go_callback, 10)
        self.yolo_go_sub  # prevent unused variable warning

        self.k_point_sub = self.create_subscription(PoseStamped, 'k_point', self.k_point_callback, 10)
        self.k_point_sub  # prevent unused variable warning

        self.k_goal_sub = self.create_subscription(Int32, 'k_goal', self.k_goal_callback, 10)
        self.k_goal_sub  # prevent unused variable warning

        self.pm_go_sub = self.create_subscription(Int32, 'pm_go', self.pm_go_callback, 10)
        self.pm_go_sub  # prevent unused variable warning

    def kick_pose_callback(self, kick_pose):
        self.val_kick_pose = kick_pose

    def yolo_go_callback(self, msg):
        self.yolo_go = msg.data

    def k_point_callback(self, k_point):
        self.val_k_point = k_point
    
    def k_goal_callback(self, msg):
        self.k_goal = msg.data

    def pm_go_callback(self, msg):
        self.pm_go = msg.data

    def nav_callback(self, msg):
        if msg.data == True:
            # Inspection route, probably read in from a file for a real application
            # from either a map or drive and repeat.
            inspection_route = [
                [3.461, -0.450], # waypoint 1
                [5.531, -0.450], # waypoint 2
                [3.461, -2.200], # waypoint 3
                [5.531, -2.200], # waypoint 4
                ]
            
            # Set our holo's initial pose
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.orientation.z = 1.0
            initial_pose.pose.orientation.w = 0.0

            # Set our holo's PMZone pose
            PMZone_pose = PoseStamped()
            PMZone_pose.header.frame_id = 'map'
            PMZone_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            PMZone_pose.pose.position.x = 0.0
            PMZone_pose.pose.position.y = 0.0
            PMZone_pose.pose.orientation.z = 1.0
            PMZone_pose.pose.orientation.w = 0.0

            # Wait for navigation to fully activate
            self.navigator.waitUntilNav2Active()

            # Send our route
            for pt in inspection_route:
                inspection_pose = PoseStamped()
                inspection_pose.header.frame_id = 'map'
                inspection_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                inspection_pose.pose.position.x = pt[0]
                inspection_pose.pose.position.y = pt[1]
                inspection_pose.pose.orientation.z = 1.0
                inspection_pose.pose.orientation.w = 0.0

                self.navigator.goToPose(inspection_pose)

                print('no yolo')

                start_time = time.time()
                while yolo_go != 1:
                    cur_time = time.time()
                    if cur_time - start_time >= 5:
                        break
                
                if yolo_go == 1:
                    yolo_pose = PoseStamped()
                    yolo_pose.header.frame_id = 'map'
                    yolo_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                    yolo_pose.pose.position.x = self.val_kick_pose.pose.position.x
                    yolo_pose.pose.position.y = self.val_kick_pose.pose.position.y
                    yolo_pose.pose.orientation.z = 1.0
                    yolo_pose.pose.orientation.w = 0.0

                    self.navigator.goToPose(yolo_pose)
                    yolo_go = 0

                    k_msg = Int32()
                    k_msg.data = 1
                    self.k_go_pub.publish(k_msg)

                    print('yolo O')

                    start_time = time.time()
                    while yolo_go != 1:
                        cur_time = time.time()
                        if cur_time - start_time >= 5:
                            break

                    if k_goal == 1:
                        k_goal_pose = PoseStamped()
                        k_goal_pose.header.frame_id = 'map'
                        k_goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                        k_goal_pose.pose.position.x = self.val_k_point.pose.position.x
                        k_goal_pose.pose.position.y = self.val_k_point.pose.position.y
                        k_goal_pose.pose.orientation.z = 1.0
                        k_goal_pose.pose.orientation.w = 0.0

                        self.navigator.goToPose(k_goal_pose)
                        k_goal = 0

                        d_msg = Int32()
                        d_msg.data = 1
                        self.d_go_pub.publish(d_msg)

                        print('k goal O')

                        start_time = time.time()
                        while yolo_go != 1:
                            cur_time = time.time()
                            if cur_time - start_time >= 60:
                                break

                        if self.pm_go == 1:
                            self.navigator.goToPose(PMZone_pose)

                            print('Finish')
                else:
                    pass
            # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
            # Simply the current waypoint ID for the demonstation
            i = 0
            while not self.navigator.isTaskComplete():
                i += 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Executing current waypoint: ' +
                        str(feedback.current_waypoint + 1) + '/' + str(len(inspection_route)))

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Inspection of shelves complete! Returning to start...')
            elif result == TaskResult.CANCELED:
                print('Inspection of shelving was canceled. Returning to start...')
            elif result == TaskResult.FAILED:
                print('Inspection of shelving failed! Returning to start...')

            # go back to start
            self.navigator.goToPose(initial_pose)

def main():
    rclpy.init()

    holo_node = holo_command()
    rclpy.spin(holo_node)

    holo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()