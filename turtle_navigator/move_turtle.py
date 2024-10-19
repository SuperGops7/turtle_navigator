#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        # Publisher to control the turtle's velocity
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Subscriber to get the turtle's current pose
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # Timer to call the move_callback function periodically
        self.timer = self.create_timer(0.1, self.move_callback)
        
        self.current_pose = Pose()
        self.target_pose = [5.0, 5.0]  # Initial target coordinates (x, y)

        # Declare parameters for dynamic adjustment
        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 4.0)

        # Add 2 obstacles
        self.obstacles = [
            {'center': [1.0, 5.5], 'size': [1.0, 1.0]},  # Obstacle 1
            {'center': [3.0, 8.0], 'size': [1.5, 1.5]}   # Obstacle 2
        ]

        self.get_logger().info("Turtle Navigator node has been started.")

    def pose_callback(self, pose):
        self.current_pose = pose

    def move_callback(self):
        msg = Twist()

        # Retrieve parameters
        linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        # Calculate the distance to the target
        distance = self.calculate_distance(
            self.current_pose.x, self.current_pose.y, 
            self.target_pose[0], self.target_pose[1]
        )

        self.get_logger().info(f"Distance to target: {distance}")

        if self.is_near_obstacle():
            self.get_logger().warn("Turtle is near an obstacle! Rerouting...")
            # Rotate the turtle to avoid the obstacle
            msg.angular.z = angular_speed  # Rotate in place
            msg.linear.x = 0.0  # Stop forward movement
        elif distance > 0.5:
            # Calculate the angle to the target
            angle_to_target = math.atan2(
                self.target_pose[1] - self.current_pose.y,
                self.target_pose[0] - self.current_pose.x
            )

            # Calculate the difference between current orientation and target angle
            angle_difference = self.normalize_angle(angle_to_target - self.current_pose.theta)

            self.get_logger().info(f"Angle difference: {angle_difference}")

            # Set angular and linear velocity
            msg.angular.z = angular_speed * angle_difference
            msg.linear.x = linear_speed * distance

            # Log the velocity command
            self.get_logger().info(f"Publishing velocity - Linear: {msg.linear.x}, Angular: {msg.angular.z}")
        else:
            self.get_logger().info(f"Reached target at {self.target_pose}")
            self.target_pose = [random.uniform(1.0, 10.0), random.uniform(1.0, 10.0)]
            self.get_logger().info(f"New target set to {self.target_pose}")

        # Publish the velocity command
        self.publisher_.publish(msg)

    def calculate_distance(self, x1, y1, x2, y2):
        """Calculate Euclidean distance between two points."""
        return math.hypot(x2 - x1, y2 - y1)

    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and +pi."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def is_near_obstacle(self):
        for obstacle in self.obstacles:
            obs_center_x, obs_center_y = obstacle['center']
            obs_width, obs_height = obstacle['size']

            # Check if the turtle's position is inside the obstacle's boundaries
            if (obs_center_x - obs_width / 2 <= self.current_pose.x <= obs_center_x + obs_width / 2 and
                    obs_center_y - obs_height / 2 <= self.current_pose.y <= obs_center_y + obs_height / 2):
                return True
        return False
def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
