#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.laser_data = None
        self.laser_subscription = self.create_subscription(
            PointCloud2,
            'laser/out',
            self.laser_callback,
            10
        )

    def laser_callback(self,msg):
        self.get_logger().info('Received data')
        self.laser_data = msg

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()