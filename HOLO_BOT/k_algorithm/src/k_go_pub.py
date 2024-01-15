#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class kgokickPublisher(Node):

    def __init__(self):
        super().__init__('k_go_pub')

        self.k_go_pub = self.create_publisher(Int32, 'k_go', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.k_go_callback)
        
    def k_go_callback(self):
        msg = Int32()
        msg.data = 1
        self.k_go_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = kgokickPublisher()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()