#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node


class kpointSubscriber(Node):

    def __init__(self):
        super().__init__('k_point_sub')

        self.k_point_sub = self.create_subscription(PoseStamped,
                                                'k_point',
                                                self.k_point_callback,
                                                10)
        
        self.k_goal_sub = self.create_subscription(Int32, 'k_goal', self.k_goal_callback, 10)
        self.k_point_sub
        self.k_goal_sub

    def k_point_callback(self, msg):
        self.get_logger().info('Received k_point: [%.2f, %.2f, %.2f]' % (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ))
    def k_goal_callback(self, msg):
        self.get_logger().info('Received k_goal: %d' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = kpointSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()