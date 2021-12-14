#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from numpy.linalg import norm
#rnmpy
from point_cloud2 import pointcloud2_to_xyz_array
from std_srvs.srv import Trigger
import math
from math import sqrt, atan2, cos, sin, asin, floor
from angles import normalize, d2r, r2d
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion


class RRTController(Node):
    def __init__(self):
        super().__init__('rrt_controller')
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
            Odometry, 'demo/odom_demo', self.odom_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, 'demo/cmd_demo',1)

    def laser_callback(self, msg):
        self.xyz_points = pointcloud2_to_xyz_array(msg)
        self.find_angles()
        self.create_occupancy_grid()

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def create_occupancy_grid(self):
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
        current_pos = self.position
        current_ori = self.orientation
        lin_vel_max = 0.4
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
        elif sqrt((current_pos.x - goal[0])**2 + (current_pos.y - goal[1])**2) > 0.6:
            cmd.linear.x = lin_vel_max
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        return cmd

    def swap(self,ta1,tb1):
        ta = tb1
        tb = ta1
        return ta, tb

    def random_config(self):
        r = np.zeros(2)
        x_dim = 50
        y_dim = 50
        r[0] = x_dim*np.random.random_sample() - (x_dim/2)
        r[1] = y_dim*np.random.random_sample() - (y_dim/2)
        return r

    def collision_checker(self,q,obstacle):
        collision = False
        # if obstacle == []:
        #     return collision
        # else:
        angle = np.linspace(0,2*np.pi,36)
        radius = 2
        x_pos = q[0]
        y_pos = q[1]
        x = radius * np.cos(angle) + x_pos
        y = radius * np.sin(angle) + y_pos
        idx = np.argsort(x)
        xs = np.empty(0)
        ys = np.empty(0)
        for i in idx:
            xs = np.append(xs,x[i])
            ys = np.append(ys,y[i])

        for i in range(0,int(len(xs)/2)):
            x_test = math.floor(xs[2*i])
            y1 = math.floor(ys[2*i])
            y2 = math.floor(ys[2*i+1])
            if (y1 > y2):
                y_test_max = y1
                y_test_min = y2
            else:
                y_test_max = y2
                y_test_min = y1
            for j in range(y_test_min, y_test_max+1):
                ind1 = x_test + 25
                ind2 = j + 25
                if ind1 >=0 and ind1 < 50 and ind2 >=0 and ind2 < 50:
                    if (obstacle[ind1,ind2] == 1):
                        collision = True
                        return collision
        return collision

    def find_nearest(self,tree,q_rand):
        length = len(tree)
        d = 100000
        for i in range(0,length):
            if len(np.shape(tree)) != 1:
                test = tree[i,0:2]
            else:
                test = tree[0:2]
            # test_w_inx = tree[i]
            # test = test_w_inx[0:2] # can probably combine these two lines
            diff = q_rand - test
            d1 = np.sqrt(np.sum(np.square(diff)))
            if d1 < d:
                q_near_pos = i
                q_near = test
                d = d1
        q_rand = np.append(q_rand, q_near_pos)
        return q_near, q_rand

    def limit(self,q_rand, q_near,step_length):
        step_size = step_length
        q_diff = q_rand[0:2] - q_near[0:2]
        q_diff = q_diff/norm(q_diff)
        q_int = q_near[0:2] + step_size*q_diff
        return q_int
    
    def local_planner(self, q_near, q_int, step_size, obstacle):
        if len(q_near) > 2 or len(q_int) > 2:
            delta_q = q_near[0:2] - q_int[0:2]
        else:
            delta_q = q_near - q_int
        num_steps = math.ceil((norm(delta_q)/step_size))
        step = delta_q/num_steps
        collision = False
        q = q_int[0:2]
        for _ in range (0,num_steps):
            q = q + step
            point_collision = self.collision_checker(q, obstacle)
            collision = collision or point_collision
        success = not collision
        return success

    def rrt_extend_single(self,tree1,q_rand,step_length,step_size,obstacle):
        q_target = np.empty(0)
        q_near, q_rand = self.find_nearest(tree1,q_rand)
        q_int = self.limit(q_rand, q_near, step_length)
        q_int = np.append(q_int,q_rand[2])
        result = self.local_planner(q_near, q_int, step_size, obstacle)
        if result == True:
            tree1 = np.vstack((tree1,q_int))
            q_target = q_int
        return result, tree1, q_target

    def rrt_extend_multiple(self, tree2, q_target, step_length, step_size, obstacle):
        q_near, q_target = self.find_nearest(tree2, q_target[0:2])
        q_int = self.limit(q_target,q_near,step_length)
        q_last = q_near[0:2]
        last_ind = q_target[2]
        num_steps = math.ceil(norm(q_target[0:2]-q_near[0:2])/step_length)
        for i in range(0,num_steps):
            result = self.local_planner(q_int,q_last,step_size,obstacle)
            if result == False:
                return result, tree2, q_connect
            q_int = np.append(q_int,last_ind)
            tree2 = np.vstack((tree2,q_int))
            last_ind = len(tree2)-1
            q_connect = q_int
            if i < num_steps:
                q_last = q_int
                q_int = self.limit(q_target,q_int,step_length)
        return result, tree2, q_connect

    def rrt_connect(self,q_start,q_goal,step_length,step_size,obstacle):
        tree1 = np.array(q_start)
        tree1 = np.append(tree1,0)
        tree2 = np.array(q_goal)
        tree2 = np.append(tree2,0)
        success = False
        for _ in range(0,1000):
            q_rand = self.random_config()
            result, tree1, q_target = self.rrt_extend_single(tree1,q_rand,step_length,step_size,obstacle)
            if result == True:
                result2, tree2, q_connect = self.rrt_extend_multiple(tree2,q_target,step_length,step_size,obstacle)
                if result2 == True:
                    success = True
                    return success, q_connect, tree1, tree2
            tree1, tree2 = self.swap(tree1,tree2)
        return success

    def findpath(self, tree1, q_connect, tree_num):
        if len(np.shape(tree1)) == 1:
            return 0
        else:
            if tree_num == 1:
                q_parent = int(tree1[-1,2])
                length = len(tree1)
                q_path = np.empty(0,dtype=int)
                q_path = np.append(q_path,length-1) #q_connect position is last point
                q_path = np.append(q_path,q_parent) #q_connects parent
                while (q_parent != 0):
                    q = tree1[q_parent]
                    q_parent = int(q[2])
                    q_path = np.append(q_path,q_parent)
                return q_path
            else:
                q_parent = int(q_connect[2])
                length = len(tree1)
                q_path = np.empty(0,dtype=int)
                q_path = np.append(q_path,length-1) #q_connect position is last point
                q_path = np.append(q_path,q_parent) #q_connects parent
                while (q_parent != 0):
                    q = tree1[q_parent]
                    q_parent = int(q[2])
                    q_path = np.append(q_path,q_parent)
                return q_path

    # def path_publisher(self, path1, tree1, obstacle):
    #     len1 = len(path1)
    #     current_pos = tree1[0]
    #     for i in path1:
    #         #get lidar data
    #         #checks for collision for next pos.
    #         next_pos = tree1[i]
    #         collision = self.collision_checker(next_pos,obstacle)
    #         if (collision == True):
    #             return collision, current_pos, obstacle
    #         #each itteration publishes a position
    #         pub = next_pos[0:2]
    #         print(pub)

    #         current_pos = tree1[i]
    #     return collision, current_pos, obstacle #False, path completed successfully.

    def get_path(self,tree1,tree2,q_connect):
        # print(self.findpath(tree1,q_connect))
        path1 = np.flip(self.findpath(tree1,q_connect,1))
        path2 = self.findpath(tree2,q_connect,2)
        path = None
        for i in path1:
            if path is None:
                path = np.array(tree1[i,0:2])
            else:
                path = np.vstack((path,tree1[i,0:2]))
        for i in path2:
            path = np.vstack((path,tree2[i,0:2]))
        return path


    # def main_loop(self,q_start,q_goal,step_length,obstacle):
    #     success, q_connect, tree1, tree2 = self.rrt_connect(q_start,q_goal,step_length,obstacle)
    #     total_success = False
    #     if (success == True):
    #         path1 = self.findpath(tree1, q_connect)
    #         path1 = path1[::-1] #flip
    #         path2 = self.findpath(tree2 ,q_connect)
    #         collision, current_pos, obstacle = self.path_publisher(path1, tree1,obstacle)
    #         if (collision == False):
    #             collision, current_pos, obstacle = self.path_publisher(path2,tree2,obstacle)
    #             if (collision == False):
    #                 total_success = True
    #                 return total_success # = true and finish
    #             else:
    #                 q_start = current_pos
    #                 return total_success, q_start, obstacle #false
    #         else:
    #             q_start = current_pos
    #             return total_success, q_start, obstacle #flase


def main(args=None):
    rclpy.init(args=args)
    rrt_controller = RRTController()
    path_unobstructed = True
    at_goal = False
    failure_counter = 0
    in_motion = False

    # wait until all initialized?
    start = np.array([8.14,-3.82])
    goal = np.array([-5,14]) # get this somehow
    step_length = 1.5
    step_size = 0.3
    obstacle = np.zeros((50, 50), dtype=np.bool)
    # generate initial plan
    path_found, q_connect, tree1, tree2 = rrt_controller.rrt_connect(start,goal,step_length,step_size,obstacle)
    path = rrt_controller.get_path(tree1,tree2,q_connect)
    current_pos_idx = 1
    print(path)
    
    while rclpy.ok():
        if in_motion:
            cmd = rrt_controller.travel_to_goal(path[current_pos_idx])
            rrt_controller.cmd_pub.publish(cmd)
            if cmd.linear.x == 0 and cmd.angular.z == 0:
                in_motion = False
                current_pos_idx = current_pos_idx + 1
        else:
            print('not in motion')
            if path_found:
                # check if at goal
                if np.all(path[current_pos_idx] == path[-1]):
                    print('at goal')
                    at_goal = True
                if not at_goal:
                    # check obstruction with local planner
                    obstacle = rrt_controller.occ_grid
                    path_unobstructed = rrt_controller.local_planner(path[current_pos_idx],path[current_pos_idx+1],step_size,obstacle)
                    if path_unobstructed:
                        print('going in motion')
                        # do motion
                        in_motion = True
                    else:
                        print('replanning')
                        # regenerate plan
                        start = path[current_pos_idx]
                        path_found, q_connect, tree1, tree2 = rrt_controller.rrt_connect(start,goal,step_length,step_size,obstacle)
                        if path_found:
                            path = rrt_controller.get_path(tree1,tree2,q_connect)
                            current_pos_idx = 0
            else:
                print('failure')
                failure_counter = failure_counter + 1
                if failure_counter >= 10:
                    break
                path_found, q_connect, tree1, tree2 = rrt_controller.rrt_connect(start,goal,step_length,step_size,obstacle)
                if path_found:
                    path = rrt_controller.get_path(tree1,tree2,q_connect)
                    current_pos_idx = 0
        rclpy.spin_once(rrt_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
