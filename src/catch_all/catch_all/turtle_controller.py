#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.tx = 5.0
        self.ty = 6.0
        self.pose_: Pose = None
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_pose(self, pose: Pose):
        self.pose_ = pose

    def control_loop(self):
        if self.pose_ == None:
            return
        
        dx = self.tx - self.pose_.x
        dy = self.ty - self.pose_.y
        distance = math.sqrt(dx**2 + dy**2)

        cmd = Twist()

        if distance > 0.1:
            cmd.linear.x = 2*distance

            ttheta = math.atan2(dy, dx)
            final = ttheta - self.pose_.theta

            if final > math.pi:
                final -= 2*math.pi
            elif final < -math.pi:
                final += 2*math.pi

            cmd.angular.z = 6*final

        else:
            cmd.linear.x = 0
            cmd.angular.z = 0

        self.vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()