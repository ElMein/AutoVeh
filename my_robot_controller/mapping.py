#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("controller started")

        self._pose_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller, 10
        )

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()
        a = 2

        self._front = min(scan.ranges[:a+1] + scan.ranges[-a:]) # Reading from the beginning and end of the list
        self._left = min(scan.ranges[90-a :90+a+1])
        self._back = min(scan.ranges[180-a :180+a+1])
        self._right = min(scan.ranges[270-a :270+a+1])

        if self._front < 1.0:
            if self._right < self._left:
                cmd.linear.x = 0.05
                cmd.angular.z = 0.5
            else:
                cmd.linear.x = 0.05
                cmd.angular.z = -0.5
        else:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
    
        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()
