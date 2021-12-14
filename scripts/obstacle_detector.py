#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
#rnmpy
from point_cloud2 import pointcloud2_to_xyz_array
from std_srvs.srv import Trigger
from math import sqrt, atan2, cos, sin, asin, floor
from angles import normalize, d2r, r2d
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion
# from geometry_msgs.msg import Pose


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.xyz_points = None
        self.occ_grid = np.zeros((50, 50), dtype=np.bool)
        self.position = Point()
        self.position.x = 0.0
        self.position.y = 0.0
        self.position.z = 0.0
        self.orientation = Quaternion()
        self.orientation.x = 0.0
        self.orientation.y = 0.0
        self.orientation.z = 0.0
        self.orientation.w = 0.0

        self.laser_subscription = self.create_subscription(
            PointCloud2, 'laser/out', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'demo/odom_demo', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'demo/cmd_demo',1)

    def laser_callback(self, msg):
        # self.get_logger().info('Received data')
        self.xyz_points = pointcloud2_to_xyz_array(msg)
        self.find_angles()
        self.create_occupancy_grid()
        # self.get_logger().info(np.str(self.xyz_points[:1,...]))
        # self.get_logger().info(np.str(poo[:1,...]))
        # self.get_logger().info(np.str(self.xyz_points))
        # self.get_logger().info(np.str(self.occ_grid))
        # self.get_logger().info(np.str(np.argwhere(self.occ_grid)))

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        # self.get_logger().info(str(self.pose.position.x))
        # self.get_logger().info(str(self.pose.position.y))
        # self.get_logger().info(str(self.pose.position.z))

    def create_occupancy_grid(self):
        # sx = 0.834015
        # sy = -0.003286
        # st = -0.012722
        # d = sqrt((sx**2) + (sy**2))
        # t = 0.626893
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
            self.occ_grid[floor(global_points[i, 0]) + 25, floor(global_points[i, 1]) + 25] = 1

    def find_angles(self):
        for i in range(len(self.xyz_points)):
            # ang = r2d(acos(
            #     self.xyz_points[i, 0]/sqrt((self.xyz_points[i, 0]**2) + (self.xyz_points[i, 1]**2))))
            ang = r2d(atan2(self.xyz_points[i, 1],self.xyz_points[i, 0]))
            self.xyz_points[i, 2] = d2r(normalize(ang, -180, 180)) # probably not necessary

    #AA
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

    def travel_to_goal(self,goal):
        # Need lin_vel, ang_vel
        current_pos = self.position
        current_ori = self.orientation
        lin_vel_max = 0.5
        ang_vel_max = 0.2
        cmd = Twist()
        dx = goal[0] - current_pos.x
        dy = goal[1] - current_pos.y
        dtheta = atan2(dy,dx)
        diff = d2r(normalize(r2d(self.euler_from_quaternion(current_ori)[2] - dtheta),-180,180))
        if abs(diff) > 0.08:
            cmd.linear.x = 0.0
            if diff >= 0.0:
                cmd.angular.z = -ang_vel_max
            else:
                cmd.angular.z = ang_vel_max
        elif sqrt((current_pos.x - goal[0])**2 + (current_pos.y - goal[1])**2) > 0.8:
            cmd.linear.x = lin_vel_max
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        return cmd
        

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    goal = np.array([6,-10])
    cmd = Twist()
    while rclpy.ok():
        # if abs(obstacle_detector.position.x - goal[0]) > 0.4 or abs(obstacle_detector.position.y - goal[1]) > 0.4:
        #     cmd = obstacle_detector.travel_to_goal(goal)
        #     obstacle_detector.cmd_pub.publish(cmd)
        # else:
        #     cmd.linear.x = 0.0
        #     cmd.angular.z = 0.0
        #     obstacle_detector.cmd_pub.publish(cmd)
        cmd = obstacle_detector.travel_to_goal(goal)
        obstacle_detector.cmd_pub.publish(cmd)
        rclpy.spin_once(obstacle_detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
