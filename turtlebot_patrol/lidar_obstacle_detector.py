import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarObstacleDetector(Node):
    """
    Subscribes to /scan and publishes sector obstacle distances.

    Partitions the LiDAR field of view into Front, Left, and Right angular
    sectors, and publishes the minimum obstacle distance for each sector on
    /obstacle_status.
    """

    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        # Declare configurable parameters
        self.declare_parameter('obstacle_distance_threshold', 0.38)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('front_angle_min_deg', -22.0)
        self.declare_parameter('front_angle_max_deg', 22.0)
        self.declare_parameter('left_angle_min_deg', 22.0)
        self.declare_parameter('left_angle_max_deg', 90.0)
        self.declare_parameter('right_angle_min_deg', -90.0)
        self.declare_parameter('right_angle_max_deg', -22.0)

        self.threshold = self.get_parameter(
            'obstacle_distance_threshold').get_parameter_value().double_value
        self.fallback_max_range = self.get_parameter(
            'max_range').get_parameter_value().double_value

        self.front_min_rad = math.radians(
            self.get_parameter('front_angle_min_deg').value)
        self.front_max_rad = math.radians(
            self.get_parameter('front_angle_max_deg').value)
        self.left_min_rad = math.radians(
            self.get_parameter('left_angle_min_deg').value)
        self.left_max_rad = math.radians(
            self.get_parameter('left_angle_max_deg').value)
        self.right_min_rad = math.radians(
            self.get_parameter('right_angle_min_deg').value)
        self.right_max_rad = math.radians(
            self.get_parameter('right_angle_max_deg').value)

        self.publisher = self.create_publisher(
            Float32MultiArray, '/obstacle_status', 10
        )
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self._logged_first_scan = False
        f_deg = (math.degrees(self.front_min_rad), math.degrees(self.front_max_rad))
        l_deg = (math.degrees(self.left_min_rad), math.degrees(self.left_max_rad))
        r_deg = (math.degrees(self.right_min_rad), math.degrees(self.right_max_rad))
        self.get_logger().info(
            f'LiDAR Obstacle Detector initialized (threshold={self.threshold:.2f}m, '
            f'front=[{f_deg[0]:.0f}°, {f_deg[1]:.0f}°], '
            f'left=[{l_deg[0]:.0f}°, {l_deg[1]:.0f}°], '
            f'right=[{r_deg[0]:.0f}°, {r_deg[1]:.0f}°])'
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize any angle in radians to [-pi, +pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_callback(self, scan: LaserScan):
        if not scan.ranges:
            return

        max_range = scan.range_max if (
            math.isfinite(scan.range_max) and scan.range_max > 0.0
        ) else self.fallback_max_range

        min_range = scan.range_min if (
            math.isfinite(scan.range_min) and scan.range_min >= 0.0
        ) else 0.12

        if not self._logged_first_scan:
            self._logged_first_scan = True
            self.get_logger().info(
                f'Received first LiDAR scan: angle_min={scan.angle_min:.2f}, '
                f'angle_max={scan.angle_max:.2f}, increment={scan.angle_increment:.4f}, '
                f'range_min={scan.range_min:.2f}, range_max={scan.range_max:.2f}, '
                f'samples={len(scan.ranges)}'
            )

        front_dists: List[float] = []
        left_dists: List[float] = []
        right_dists: List[float] = []

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        for i, distance in enumerate(scan.ranges):
            # Calculate beam angle and normalize to [-pi, pi]
            raw_angle = angle_min + i * angle_inc
            angle = self.normalize_angle(raw_angle)

            # Check validity
            if math.isfinite(distance):
                if min_range <= distance <= max_range:
                    valid_dist = distance
                elif distance < min_range:
                    # Reading too close or zero-return; treat as no-return / clear
                    continue
                else:
                    valid_dist = max_range
            elif math.isinf(distance) and distance > 0.0:
                # Infinite reading = no obstacle detected up to max_range
                valid_dist = max_range
            else:
                # NaN or negative reading
                continue

            # Assign to sector
            if self.front_min_rad <= angle <= self.front_max_rad:
                front_dists.append(valid_dist)
            elif self.left_min_rad <= angle <= self.left_max_rad:
                left_dists.append(valid_dist)
            elif self.right_min_rad <= angle <= self.right_max_rad:
                right_dists.append(valid_dist)

        # Compute sector minimum distances, safely defaulting to max_range
        front_val = float(min(front_dists)) if front_dists else max_range
        left_val = float(min(left_dists)) if left_dists else max_range
        right_val = float(min(right_dists)) if right_dists else max_range

        msg = Float32MultiArray()
        msg.data = [front_val, left_val, right_val]
        self.publisher.publish(msg)


def main(args=None):
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
