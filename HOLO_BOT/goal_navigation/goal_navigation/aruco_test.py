from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node


class aruco_test(Node):

    def __init__(self):
        super().__init__('poseNav')
        self.docking_pub = self.create_publisher(Int32, '/fin_docking', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tracer_cmd = Twist()
        self.docking_msg = Int32()
        self.arucoFlag = False
        self.arucoMove = True

        self.aruco_sub = self.create_subscription(PoseArray,
                                                '/aruco_poses',
                                                self.aruco_detect_callback,
                                                1)
        
        self.navsub = self.create_subscription(
            Bool,
            '/lvbool',
            self.goalCallback,
            1)
        
    def goalCallback(self, cmd):
        self.startCmd = cmd.data
        self.navigationtoPose()
        
    def aruco_detect_callback(self, aruco_data):

        if aruco_data is not None:
            self.arucoPose = aruco_data.poses[0].position
            self.arucoOrientation = aruco_data.poses[0].orientation
            self.arucoDist = round(
                self.arucoPose.z, 2)
            self.arucoOrnt = round(
                self.arucoPose.x, 2)
            self.arucoFlag = True

        
    def arucoMove(self):
        while self.arucoDist < 0.5:
            print(self.arucoDist)


    def navigationtoPose(self):                
        
        try:
            print("flag: ", self.arucoFlag)
            print("move: ", self.arucoMove)
            print("Dist: ", self.arucoDist)
            print("Ornt: ", self.arucoOrnt)

            if self.arucoFlag == True:
                if self.arucoDist > 0.112 and self.arucoMove == True:
                    self.tracer_cmd.linear.x = -0.10
                    if self.arucoOrnt < 0.1:
                        self.tracer_cmd.angular.z = +0.10
                    if self.arucoOrnt > 0.0:
                        self.tracer_cmd.angular.z = -0.10
                    self.cmd_pub.publish(self.tracer_cmd)
                elif self.arucoDist < 0.112:
                    self.docking_msg.data = 1
                    print("Reached")
                    # self.arucoMove = False
                    self.tracer_cmd.linear.x = 0.0
                    self.tracer_cmd.angular.z = 0.0
                    self.cmd_pub.publish(self.tracer_cmd)
                    self.docking_pub.publish(self.docking_msg)
            
            else:
                print("else!")

        except:
            pass


def main():
    rclpy.init()
    poseNav = aruco_test()

    rclpy.spin(poseNav)

    poseNav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
