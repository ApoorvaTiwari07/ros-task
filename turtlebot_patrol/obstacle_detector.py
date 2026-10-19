"""Backward-compatibility alias for lidar_obstacle_detector."""
from turtlebot_patrol.lidar_obstacle_detector import LidarObstacleDetector, main

ObstacleDetector = LidarObstacleDetector
__all__ = ['ObstacleDetector', 'LidarObstacleDetector', 'main']

if __name__ == '__main__':
    main()
