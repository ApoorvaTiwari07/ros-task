import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class TelemetryLogger(Node):
    """
    Subscribes to /odom and /cmd_vel and logs robot telemetry.

    Calculates cumulative distance travelled, average linear speed, and battery
    drain during motion. Publishes formatted telemetry to /robot_telemetry
    (std_msgs/msg/String) and records timestamped entries to a CSV file.
    """

    def __init__(self):
        super().__init__('telemetry_logger')

        # Declare configurable parameters
        self.declare_parameter('telemetry_period', 1.0)
        self.declare_parameter('initial_battery', 100.0)
        self.declare_parameter('battery_drain_rate', 0.1)  # % drain per second of motion
        self.declare_parameter(
            'csv_file_path',
            os.path.expanduser('~/robot_ws/telemetry_log.csv')
        )

        self.period = self.get_parameter('telemetry_period').value
        self.battery = float(self.get_parameter('initial_battery').value)
        self.drain_rate = float(self.get_parameter('battery_drain_rate').value)
        self.csv_path = os.path.expanduser(
            self.get_parameter('csv_file_path').value)

        # Odometry tracking
        self.total_distance = 0.0
        self.last_position = None
        self.motion_time_sec = 0.0
        self.last_update_time = time.time()

        # Motion status
        self.is_moving = False
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

        # Subscriptions and Publishers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.publisher = self.create_publisher(
            String, '/robot_telemetry', 10
        )

        # CSV initialization
        csv_dir = os.path.dirname(self.csv_path)
        if csv_dir and not os.path.exists(csv_dir):
            try:
                os.makedirs(csv_dir, exist_ok=True)
            except OSError:
                self.csv_path = 'telemetry_log.csv'

        try:
            self.csv_file = open(self.csv_path, 'w', newline='', buffering=1)
        except OSError:
            self.csv_path = 'telemetry_log.csv'
            self.csv_file = open(self.csv_path, 'w', newline='', buffering=1)
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp', 'distance_m', 'average_speed_m_s', 'battery_percent'
        ])
        self.csv_file.flush()

        self.timer = self.create_timer(self.period, self.log_telemetry)
        self.get_logger().info(
            f'Telemetry Logger started. Logging every {self.period:.1f}s to {self.csv_path}'
        )

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        curr = (pos.x, pos.y)

        if self.last_position is not None:
            dx = curr[0] - self.last_position[0]
            dy = curr[1] - self.last_position[1]
            step_dist = math.hypot(dx, dy)
            self.total_distance += step_dist

        self.last_position = curr

    def cmd_callback(self, msg: Twist):
        self.current_linear_speed = abs(msg.linear.x)
        self.current_angular_speed = abs(msg.angular.z)
        self.is_moving = (
            self.current_linear_speed > 0.005 or
            self.current_angular_speed > 0.01
        )

    def log_telemetry(self):
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        # Update battery and motion time only when robot is moving
        if self.is_moving:
            self.motion_time_sec += dt
            self.battery = max(0.0, self.battery - self.drain_rate * dt)

        # Average linear speed = total distance / active motion time
        avg_speed = (
            self.total_distance / self.motion_time_sec
            if self.motion_time_sec > 0.1 else 0.0
        )

        # Publish String message
        telemetry_str = (
            f"Distance: {self.total_distance:.2f} m | "
            f"Avg Speed: {avg_speed:.2f} m/s | "
            f"Battery: {self.battery:.1f}% | "
            f"Moving: {self.is_moving}"
        )
        msg = String()
        msg.data = telemetry_str
        self.publisher.publish(msg)

        # Write to CSV
        self.writer.writerow([
            f"{now:.2f}",
            f"{self.total_distance:.3f}",
            f"{avg_speed:.3f}",
            f"{self.battery:.2f}"
        ])
        self.csv_file.flush()

        self.get_logger().info(f"[Telemetry] {telemetry_str}")

    def destroy_node(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
