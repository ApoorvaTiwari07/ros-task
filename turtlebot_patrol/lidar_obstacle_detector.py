#!/usr/bin/env python3
"""
lidar_obstacle_detector.py

Perception node for TurtleBot3.
- Subscribes to /scan (sensor_msgs/msg/LaserScan)
- Partitions 360-degree LiDAR scan into Front (-30 to +30 deg),
  Left (+30 to +90 deg), and Right (-90 to -30 deg) sectors.
- Computes minimum distance for each sector.
- Publishes sector distances and obstacle flag on /obstacle_status.
- Emits visualization markers on /obstacle_markers for RViz display.
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class LidarObstacleDetector(Node):
    """Perception node partitioning LiDAR scans and reporting sector hazards."""

    def __init__(self) -> None:
        super().__init__('lidar_obstacle_detector')

        # Declare parameters
        self.declare_parameter('safety_distance_threshold', 0.5)
        self.declare_parameter('front_angle_min_deg', -30.0)
        self.declare_parameter('front_angle_max_deg', 30.0)
        self.declare_parameter('left_angle_min_deg', 30.0)
        self.declare_parameter('left_angle_max_deg', 90.0)
        self.declare_parameter('right_angle_min_deg', -90.0)
        self.declare_parameter('right_angle_max_deg', -30.0)
        self.declare_parameter('publish_markers', True)

        # Retrieve parameters
        self.safety_threshold = float(
            self.get_parameter('safety_distance_threshold').value
        )
        self.front_min_rad = math.radians(
            float(self.get_parameter('front_angle_min_deg').value)
        )
        self.front_max_rad = math.radians(
            float(self.get_parameter('front_angle_max_deg').value)
        )
        self.left_min_rad = math.radians(
            float(self.get_parameter('left_angle_min_deg').value)
        )
        self.left_max_rad = math.radians(
            float(self.get_parameter('left_angle_max_deg').value)
        )
        self.right_min_rad = math.radians(
            float(self.get_parameter('right_angle_min_deg').value)
        )
        self.right_max_rad = math.radians(
            float(self.get_parameter('right_angle_max_deg').value)
        )
        self.publish_markers = bool(
            self.get_parameter('publish_markers').value
        )

        # Publishers
        self.obstacle_pub = self.create_publisher(
            Float32MultiArray, '/obstacle_status', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/obstacle_markers', 10
        )

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.get_logger().info(
            f'LidarObstacleDetector initialized. Safety threshold: {self.safety_threshold}m'
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize an angle into [-pi, +pi] interval."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def compute_sector_min(
        self,
        scan: LaserScan,
        angle_min: float,
        angle_max: float
    ) -> float:
        """
        Calculate minimum valid distance in [angle_min, angle_max].
        Handles angle wrapping across [-pi, pi].
        """
        valid_ranges: List[float] = []
        angle_increment = scan.angle_increment
        start_angle = scan.angle_min

        # If sector wraps around 0/bounds, determine bounds logic
        is_crossing_zero = (angle_min > angle_max)

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            raw_angle = start_angle + i * angle_increment
            norm_angle = self.normalize_angle(raw_angle)

            if is_crossing_zero:
                in_sector = (norm_angle >= angle_min or norm_angle <= angle_max)
            else:
                in_sector = (angle_min <= norm_angle <= angle_max)

            if in_sector:
                valid_ranges.append(distance)

        return min(valid_ranges) if valid_ranges else float('inf')

    def scan_callback(self, scan: LaserScan) -> None:
        """Process incoming LaserScan message."""
        # Calculate minimum distance in Front, Left, and Right sectors
        front_min = self.compute_sector_min(scan, self.front_min_rad, self.front_max_rad)
        left_min = self.compute_sector_min(scan, self.left_min_rad, self.left_max_rad)
        right_min = self.compute_sector_min(scan, self.right_min_rad, self.right_max_rad)

        # Flag an obstacle when the relevant distance is below the safety threshold
        obstacle_flag = 1.0 if (front_min < self.safety_threshold) else 0.0

        # Publish on /obstacle_status: [front, left, right, obstacle_flag]
        msg = Float32MultiArray()
        msg.data = [
            float(front_min),
            float(left_min),
            float(right_min),
            float(obstacle_flag)
        ]
        self.obstacle_pub.publish(msg)

        if self.publish_markers:
            self.publish_rviz_markers(scan.header.frame_id or 'base_scan', front_min, obstacle_flag > 0.5)

    def publish_rviz_markers(self, frame_id: str, front_dist: float, obstacle_flag: bool) -> None:
        """Publish RViz markers for visualization of detected obstacles."""
        marker_array = MarkerArray()

        # Marker 1: Obstacle warning sphere in front of the robot
        sphere_marker = Marker()
        sphere_marker.header.frame_id = frame_id
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = 'obstacle_indicator'
        sphere_marker.id = 0
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD

        # Distance representation (clamped for visual display)
        disp_dist = min(front_dist, 2.5) if math.isfinite(front_dist) else 2.5
        sphere_marker.pose.position.x = float(disp_dist)
        sphere_marker.pose.position.y = 0.0
        sphere_marker.pose.position.z = 0.1
        sphere_marker.pose.orientation.w = 1.0

        sphere_marker.scale.x = 0.3
        sphere_marker.scale.y = 0.3
        sphere_marker.scale.z = 0.3

        if obstacle_flag:
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.1
            sphere_marker.color.b = 0.1
            sphere_marker.color.a = 0.9  # Red warning
        else:
            sphere_marker.color.r = 0.1
            sphere_marker.color.g = 0.9
            sphere_marker.color.b = 0.1
            sphere_marker.color.a = 0.4  # Green clear

        marker_array.markers.append(sphere_marker)
        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
