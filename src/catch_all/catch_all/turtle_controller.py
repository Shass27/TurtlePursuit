#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math
from interfaces.msg import Turtle
from interfaces.msg import Turtlearray
from turtlesim.srv import Kill
from functools import partial

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_: Pose = None
        self.turtle_to_catch: Turtle = None
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
        self.vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.spawn_list_sub = self.create_subscription(Turtlearray, "turtles", self.callback_turtle_no, 10)
        self.kill_client = self.create_client(Kill, "/kill")
        while not self.kill_client.wait_for_service(1):
            self.get_logger().warn("Waiting for /kill service")
        self.turt_counter = 0
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_pose(self, pose: Pose):
        self.pose_ = pose

    def callback_turtle_no(self, msg: Turtlearray):
        if len(msg.turtles)>0 and self.turt_counter<len(msg.turtles):
            self.turtle_to_catch = msg.turtles[self.turt_counter]

    def call_to_kill(self, name):
        request = Kill.Request()
        request.name = name

        future = self.kill_client.call_async(request)
        future.add_done_callback(partial(self.callback_kill, name=name))

    def callback_kill(self, future, name):
        if future.result():        
            self.get_logger().info(name+ " has been killed")
        else:
            self.get_logger().error("Failed to kill "+name)

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch == None:
            return
        
        dx = self.turtle_to_catch.x - self.pose_.x
        dy = self.turtle_to_catch.y - self.pose_.y
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
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_to_kill(self.turtle_to_catch.name)
            self.turt_counter+=1
            self.get_logger().info(self.turtle_to_catch.name + " has been chased")
            self.turtle_to_catch=None

        self.vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()