#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
import tf_transformations
import time
import math

class TurtleNavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("our navigation is started")
        self.goal_poses = [] # List to store goal poses
        self.current_goal_index = 0

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, "/goal_pose", 10)

        self.odom_listener = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)

        ############# [Initial Location] ############
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        
        qq = tf_transformations.quaternion_from_euler(0,0,0)# x, y, z or Roll Pitch Yaw
        initial_pose.pose.pose.orientation.x = qq[0]
        initial_pose.pose.pose.orientation.y = qq[1]
        initial_pose.pose.pose.orientation.z = qq[2]
        initial_pose.pose.pose.orientation.w = qq[3]
        self.initial_pose_publisher.publish(initial_pose)

        #################################
        # Identify the difference between home and map reference frame
        # Due to the complexity of various planes, we use the first odometry readback to initialize our home position.
        # This will minimize guesswork on our end.
        self.x_home = None
        self.y_home = None

        # Initialize goal poses as dictionaries {x, y, w}
        self.goal_poses.append({'x': 1.3, 'y': 0.9, 'yaw': -30})
        self.goal_poses.append({'x': 2.0, 'y': 1.5, 'yaw': 60})
        time.sleep(5)
        self.publish_goal()

        self.distance_log_counter = 0

    def odom_callback(self, msg: Odometry):
    # Check if current goal pose is reached
        current_pose = msg.pose.pose

        if self.x_home is None:
            self.x_home = current_pose.position.x
            self.y_home = current_pose.position.y
            self.get_logger().info("Initialized home to x: {} y: {}".format(self.x_home, self.y_home))

        goal_pose = self.goal_poses[self.current_goal_index]
        distance_to_goal = (((current_pose.position.x - self.x_home) - goal_pose['x']) ** 2 +
                            ((current_pose.position.y - self.y_home) - goal_pose['y']) ** 2) ** 0.5

        if distance_to_goal < 0.6: # You can adjust this threshold
            self.get_logger().info("Distance within threshold. {}".format(distance_to_goal))
            self.publish_next_goal()
        else:
            self.distance_log_counter += 1
            if self.distance_log_counter == 10:
                self.distance_log_counter = 0
                self.get_logger().info("Distance to goal: {}; pose: {}".format(distance_to_goal, current_pose.position))

    def publish_next_goal(self):
        # Check if there are more goals to explore
        if self.current_goal_index < len(self.goal_poses) - 1:
            self.current_goal_index += 1
            self.publish_goal()
        else:
            self.get_logger().info("All goals explored!")
            self.stop()

    def publish_goal(self):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.goal_poses[self.current_goal_index]['x']
        pose_msg.pose.position.y = self.goal_poses[self.current_goal_index]['y']
        yaw_angle = self.goal_poses[self.current_goal_index]['yaw']
        qq = tf_transformations.quaternion_from_euler(0,0,yaw_angle)
        
        pose_msg.pose.orientation.x = qq[0]
        pose_msg.pose.orientation.y = qq[1]
        pose_msg.pose.orientation.z = qq[2]
        pose_msg.pose.orientation.w = qq[3]
        
        pose_msg.header.frame_id = 'map'
        self.goal_pose_publisher.publish(pose_msg)
        self.get_logger().info("Published goal: {}".format(self.current_goal_index))

    def stop(self):
        self.get_logger().info("Stopping the node.")
        rclpy.shutdown()
        raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigationNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt):
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
