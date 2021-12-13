#!/usr/bin/env python3
from numpy.core.numeric import roll
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from point_cloud2 import pointcloud2_to_xyz_array
from std_srvs.srv import Trigger
from math import sqrt, atan2, cos, sin, asin, floor
from angles import normalize, d2r, r2d
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.xyz_points = None
        self.occ_grid = np.zeros((50, 50), dtype=np.bool)
        self.position = None
        self.orientation = None
        self.srv = self.create_service(
            Trigger, 'obstacle_update', self.obstacle_callback)
        self.laser_subscription = self.create_subscription(
            PointCloud2, 'laser/out', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'demo/odom_demo', self.odom_callback, 10)
        # self.obstacle_pub = self.create_publisher()

    def laser_callback(self, msg):
        # self.get_logger().info('Received data')
        self.xyz_points = pointcloud2_to_xyz_array(msg)
        self.find_angles()
        poo = self.tester_fn()
        # self.get_logger().info(np.str(self.xyz_points[:1,...]))
        # self.get_logger().info(np.str(poo[:1,...]))
        self.get_logger().info(np.str(self.xyz_points))
        self.get_logger().info(np.str(poo))
        # self.get_logger().info(np.str(np.argwhere(self.occ_grid)))

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        # self.get_logger().info(str(self.pose.position.x))
        # self.get_logger().info(str(self.pose.position.y))
        # self.get_logger().info(str(self.pose.position.z))

    def tester_fn(self):
        # sx = 0.834015
        # sy = -0.003286
        # st = -0.012722
        # d = sqrt((sx**2) + (sy**2))
        # t = 0.626893
        robot_pose = np.array([0, 0, 0]) # Have to get this somehow
        if self.position is not None:
            robot_pose = np.array([self.position.x, self.position.y, self.euler_from_quaternion(self.orientation)[2]])
        robot_x = robot_pose[0]
        robot_y = robot_pose[1]
        robot_theta = robot_pose[2]
        global_points = np.zeros_like(self.xyz_points[...,:2])
        for i in range(len(self.xyz_points)):
            r = sqrt((self.xyz_points[i, 0]**2) + (self.xyz_points[i, 1]**2))
            global_points[i, 0] = robot_x + r*cos(robot_theta + self.xyz_points[i, 2])
            global_points[i, 1] = robot_y + r*sin(robot_theta + self.xyz_points[i, 2])
        for i in range(len(global_points)):
            self.occ_grid[floor(global_points[i, 0]) - 25, floor(global_points[i, 1]) - 25] = 1
        return global_points

    def obstacle_callback(self, request, response):
        robot_pose = np.array([0, 0, 0]) # Have to get this somehow
        robot_x = robot_pose[0]
        robot_y = robot_pose[1]
        robot_theta = robot_pose[2]
        global_points = np.zeros_like(self.xyz_points[...,:2])
        for i in range(len(self.xyz_points)):
            global_points[i, 0] = robot_x*cos(robot_theta) + self.xyz_points[i, 0]*cos(robot_theta + self.xyz_points[i, 2])
            global_points[i, 1] = robot_y*sin(robot_theta) + self.xyz_points[i, 1]*sin(robot_theta + self.xyz_points[i, 2])
        

        # Find the vertices and publish them
        # prev_r = 0
        # prev_dr = 0
        # for i in range(len(self.xyz_points)):
        #     r = sqrt(self.xyz_points[i, 0]**2 + self.xyz_points[i, 1]**2)
        #     dr = r - prev_r
        #     if np.sign(dr) != np.sign(prev_dr):
        #         self.vertices = np.concatenate(self.vertices, self.xyz_points[i-1])
        #         continue

        #     prev_r = r
        #     prev_dr = dr
        return

    def find_angles(self):
        for i in range(len(self.xyz_points)):
            # ang = r2d(acos(
            #     self.xyz_points[i, 0]/sqrt((self.xyz_points[i, 0]**2) + (self.xyz_points[i, 1]**2))))
            ang = r2d(atan2(self.xyz_points[i, 1],self.xyz_points[i, 0]))
            self.xyz_points[i, 2] = d2r(normalize(ang, -180, 180)) # probably not necessary

    def euler_from_quaternion(self,orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        t0 = +2.0*(w*x + y*z)
        t1 = +1.0 - 2.0*(x*x + y*y)
        r = atan2(t0, t1)
        t2 = +2.0*(w*y - z*x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        p = asin(t2)
        t3 = +2.0*(w*z + x*y)
        t4 = +1.0 - 2.0*(y*y + z*z)
        yaw = atan2(t3, t4)
        return np.array([r, p, yaw])


def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
