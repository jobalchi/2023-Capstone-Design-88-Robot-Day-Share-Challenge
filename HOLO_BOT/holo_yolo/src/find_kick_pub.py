#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class findkickPublisher(Node):

    def __init__(self):
        super().__init__('find_kick_pub')

        self.find_kick_pub = self.create_publisher(Int32, 'find_kick', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.find_kick_callback)
        
    def find_kick_callback(self):
        msg = Int32()
        msg.data = 1
        self.find_kick_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = findkickPublisher()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()