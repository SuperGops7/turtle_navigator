# Turtle Navigator - ROS2 Autonomous Navigation with Turtlesim

## Overview

This ROS2 package demonstrates autonomous navigation using Turtlesim. The turtle navigates to random waypoints within the Turtlesim environment, adjusting its orientation and speed dynamically to reach each target.

## Features

- Autonomous waypoint navigation
- Dynamic parameter tuning for linear and angular speed
- Subscription to pose data for real-time navigation feedback

## Installation

1. Clone this repository into your ROS2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone <your-repo-url>
    ```

2. Build the package:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3. Source the workspace:
    ```bash
    source install/setup.bash
    ```

## Running the Simulation

1. Start the Turtlesim node:
    ```bash
    ros2 run turtlesim turtlesim_node
    ```

2. Run the `move_turtle` node:
    ```bash
    ros2 run turtle_navigator move_turtle
    ```

## Customization

- Change the turtle’s linear and angular speed dynamically with ROS2 parameters:
    ```bash
    ros2 param set /move_turtle linear_speed 1.0
    ros2 param set /move_turtle angular_speed 2.0
    ```

