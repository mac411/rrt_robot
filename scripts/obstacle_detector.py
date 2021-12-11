#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from point_cloud2 import pointcloud2_to_xyz_array
from std_srvs.srv import Trigger


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.xyz_points = None
        self.srv = self.create_service(
            Trigger, 'obstacle_update', self.obstacle_callback)
        self.laser_subscription = self.create_subscription(
            PointCloud2, 'laser/out', self.laser_callback, 10)
        self.obstacle_pub = self.create_publisher()

    def laser_callback(self, msg):
        # self.get_logger().info('Received data')
        self.xyz_points = pointcloud2_to_xyz_array(msg)
        # self.get_logger().info(np.str(self.xyz_points))

    def obstacle_callback(self, request, response):
        # Find the vertices and publish them
        return


def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
