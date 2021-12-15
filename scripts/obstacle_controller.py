#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ObstacleController(Node):
    def __init__(self):
        super().__init__('obstacle_controller')
        self.cmd1_pub = self.create_publisher(Twist, 'obstacle1/cmd_vel', 1)
        self.cmd2_pub = self.create_publisher(Twist, 'obstacle2/cmd_vel', 1)
        self.cmd3_pub = self.create_publisher(Twist, 'obstacle3/cmd_vel', 1)
        self.cmd1 = Twist()
        self.cmd2 = Twist()
        self.cmd3 = Twist()
        self.direction = 1.0
        timer_period = 20
        self.timer = self.create_timer(timer_period,self.publish_commands)
    
    def publish_commands(self):
        self.cmd1.linear.x = self.direction*0.4
        self.cmd1.linear.y = self.direction*-0.4
        self.cmd1_pub.publish(self.cmd1)

        self.cmd2.linear.x = self.direction*-0.4
        self.cmd2.linear.y = self.direction*0.4
        self.cmd2_pub.publish(self.cmd2)

        self.cmd3.linear.x = self.direction*0.4
        self.cmd3.linear.y = self.direction*-0.4
        self.cmd3_pub.publish(self.cmd3)

        self.direction = -1*self.direction



def main(args=None):
    rclpy.init(args=args)
    obstacle_controller = ObstacleController()
    rclpy.spin(obstacle_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
