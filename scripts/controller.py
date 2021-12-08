#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello")

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()