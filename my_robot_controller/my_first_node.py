#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
  def __init__(self):
    super().__init__("first_node") # Real Node Name
    self.get_logger().info("Hellooooo")

def main(args=None):
  rclpy.init(args=args) # we initialize ROS2 communications
  node = MyNode()
  rclpy.shutdown() # we will close the node

if __name__ == '__main__':
  main()
