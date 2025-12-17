#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
import random
import math
from interfaces.msg import Turtle
from interfaces.msg import Turtlearray

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.counter = 2
        self.declare_parameter("prefix", "turtle")
        self.turtle_prefix = self.get_parameter("prefix").value
        self.spawn_client = self.create_client(Spawn, "/spawn")
        self.spawn_list_pub = self.create_publisher(Turtlearray, "turtles", 10)
        self.TurtleList = []
        self.declare_parameter("freq", 1.0)
        self.spawn_freq = self.get_parameter("freq").value
        self.spawn_timer = self.create_timer(self.spawn_freq, self.spawn)

    def spawn(self):
        name = self.turtle_prefix + str(self.counter)
        self.counter+=1
        x=random.uniform(0,11)
        y=random.uniform(0,11)
        theta=random.uniform(0, 2*math.pi)
        self.call_spawn_service(name, x, y, theta)

    def call_spawn_service(self, turtle_name, x, y, theta):
        while not self.spawn_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /spawn service")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn, request=request))

    def callback_spawn(self, future, request):
        response: Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info("New alive turtle: " + response.name)

            new_turtle = Turtle()
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            new_turtle.name = response.name
            self.TurtleList.append(new_turtle)
            msg = Turtlearray()
            msg.turtles = self.TurtleList
            self.spawn_list_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()