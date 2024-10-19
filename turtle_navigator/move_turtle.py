#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
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

        # Define obstacles as turtles
        self.obstacles = [
            {'name': 'turtle2', 'position': [1.0, 7.0]},  # Obstacle Turtle 1
            {'name': 'turtle3', 'position': [7.0, 2.0]}   # Obstacle Turtle 2
        ]

        # Spawn obstacle turtles at defined positions
        for obstacle in self.obstacles:
            self.spawn_obstacle_turtle(obstacle['name'], obstacle['position'][0], obstacle['position'][1])

        self.get_logger().info("Turtle Navigator node has been started.")

    def spawn_obstacle_turtle(self, name, x, y):
        """Spawn additional turtles as obstacles."""
        client = self.create_client(Spawn, '/spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn not available, waiting...')
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle {response.name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def pose_callback(self, pose):
        self.current_pose = pose

    def get_obstacle_positions(self):
        """Get positions of the obstacle turtles."""
        obstacle_positions = []
        for obstacle in self.obstacles:
            try:
                pose_topic = f'/{obstacle["name"]}/pose'
                msg = self.get_latest_pose(pose_topic)
                obstacle_positions.append((msg.x, msg.y))
            except Exception as e:
                self.get_logger().warn(f"Could not retrieve pose for {obstacle['name']}: {e}")
        return obstacle_positions
    
    def get_latest_pose(self, topic_name):
        """Retrieve the latest pose from the given topic."""
        return next(self.create_subscription(Pose, topic_name, lambda msg: msg, 10))

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
            self.target_pose = self.get_random_target()
            self.get_logger().info(f"New target set to {self.target_pose}")

        # Publish the velocity command
        self.publisher_.publish(msg)

    def get_random_target(self):
        pose_x, pose_y = random.uniform(1.1, 9.9), random.uniform(1.1, 9.9)
        obstacle_positions = self.get_obstacle_positions()
        for obs_x, obs_y in obstacle_positions:
            while pose_x == obs_x and pose_y == obs_y:
                self.get_logger().info(f"Changing position of the target")
                pose_x, pose_y = random.uniform(1.1, 9.9), random.uniform(1.1, 9.9)
        return [pose_x, pose_y]

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
        """Check if the main turtle is near any obstacle turtles."""
        obstacle_positions = self.get_obstacle_positions()
        for obs_x, obs_y in obstacle_positions:
            # If the main turtle is close to the obstacle (within 1 unit)
            if self.calculate_distance(self.current_pose.x, self.current_pose.y, obs_x, obs_y) < 1.0:
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
