# Turtle Navigator - ROS2 Autonomous Navigation

![Python](https://img.shields.io/badge/python-3.10-blue.svg)
![Ubuntu](https://img.shields.io/badge/ubuntu-22.04-orange.svg)
![ROS](https://img.shields.io/badge/ros-humble-blue.svg)

## Overview

This ROS2 package demonstrates autonomous navigation using Turtlesim. The turtle navigates to random waypoints and avoids obstacles represented by additional turtles in the environment.

## Features

- Autonomous waypoint navigation
- Dynamic speed control using ROS2 parameters
- Obstacle avoidance with real-time feedback
- Modular design with separate navigation and obstacle management

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

- Adjust turtle speed:
    ```bash
    ros2 param set /move_turtle linear_speed 1.0
    ros2 param set /move_turtle angular_speed 2.0
    ```

- Add obstacles in `turtle_obstacle_manager.py` by defining additional turtles:
    ```python
    self.obstacles = [
        {'name': 'turtle2', 'position': [1.0, 7.0]},
        {'name': 'turtle3', 'position': [7.0, 2.0]}
    ]
    ```

## Class Overview

- **`MoveTurtle`**: Manages turtle navigation and obstacle avoidance.
- **`TurtleObstacleManager`**: Handles spawning and tracking of obstacle turtles.

## Future Improvements

- Add dynamic obstacles or implement path planning for smarter navigation.
