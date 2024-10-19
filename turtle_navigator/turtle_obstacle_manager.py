from turtlesim.srv import Spawn


class TurtleObstacleManager:
    def __init__(self) -> None:
        # Define obstacles as turtles
        self.obstacles = [
            {"name": "turtle2", "position": [1.0, 7.0]},  # Obstacle Turtle 1
            {"name": "turtle3", "position": [7.0, 2.0]},  # Obstacle Turtle 2
        ]

        # Spawn obstacle turtles at defined positions
        for obstacle in self.obstacles:
            self.spawn_obstacle_turtle(
                obstacle["name"], obstacle["position"][0], obstacle["position"][1]
            )

    def spawn_obstacle_turtle(self, name, x, y):
        """Spawn additional turtles as obstacles."""
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service /spawn not available, waiting...")

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
