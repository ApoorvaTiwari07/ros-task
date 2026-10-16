#!/usr/bin/env python3
"""
telemetry_logger.py

Diagnostics & Logging node for TurtleBot3.
- Subscribes to /odom, /cmd_vel, and /estop_state.
- Calculates total distance travelled from odometry.
- Calculates average linear speed.
- Simulates battery drain (1% per 10 seconds of motion starting from 100%).
- Publishes /robot_telemetry (std_msgs/msg/String).
- Prints telemetry to console and periodically logs to telemetry_log.csv.
"""

import csv
import json
import math
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class TelemetryLogger(Node):
    """Subscribes to robot motion feeds, computes statistics, and logs telemetry."""

    def __init__(self) -> None:
        super().__init__('telemetry_logger')

        # Parameters
        self.declare_parameter('log_rate', 1.0)                      # Hz for CSV logging
        self.declare_parameter('console_log_rate', 0.5)              # Hz for console print (every 2s)
        self.declare_parameter('csv_log_path', 'telemetry_log.csv')
        self.declare_parameter('battery_start_percentage', 100.0)
        self.declare_parameter('seconds_per_battery_percent', 10.0)
        self.declare_parameter('motion_linear_threshold', 0.01)
        self.declare_parameter('motion_angular_threshold', 0.02)

        self.csv_log_path = str(self.get_parameter('csv_log_path').value)
        self.battery_percent = float(self.get_parameter('battery_start_percentage').value)
        self.seconds_per_pct = float(self.get_parameter('seconds_per_battery_percent').value)
        self.linear_thresh = float(self.get_parameter('motion_linear_threshold').value)
        self.angular_thresh = float(self.get_parameter('motion_angular_threshold').value)
        log_rate = float(self.get_parameter('log_rate').value)
        console_rate = float(self.get_parameter('console_log_rate').value)

        # Telemetry variables
        self.total_distance = 0.0
        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None
        self.current_speed = 0.0
        self.current_angular_speed = 0.0
        self.total_motion_time = 0.0
        self.is_moving = False
        self.estop_active = False

        # Time tracking
        self.last_update_time = self.get_clock().now()
        self.last_console_time = self.get_clock().now()

        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/robot_telemetry', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/estop_state', self.estop_callback, 10)

        # Initialize CSV file with header if it doesn't exist
        self.init_csv_file()

        # Timer for periodic logging and publishing
        self.timer = self.create_timer(1.0 / max(log_rate, 0.1), self.timer_callback)

        self.get_logger().info(
            f'TelemetryLogger active. Logging to {os.path.abspath(self.csv_log_path)}'
        )

    def init_csv_file(self) -> None:
        """Create CSV file and write header row if not already present."""
        try:
            file_exists = os.path.exists(self.csv_log_path) and os.path.getsize(self.csv_log_path) > 0
            with open(self.csv_log_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    writer.writerow([
                        'timestamp_sec',
                        'distance_travelled_m',
                        'current_speed_mps',
                        'avg_speed_mps',
                        'battery_percent',
                        'motion_time_sec',
                        'is_moving',
                        'estop_active'
                    ])
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CSV log: {e}')

    def odom_callback(self, msg: Odometry) -> None:
        """Track distance travelled from odometry updates."""
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y

        if self.prev_x is not None and self.prev_y is not None:
            delta_dist = math.hypot(curr_x - self.prev_x, curr_y - self.prev_y)
            # Filter noise and large teleports
            if 0.0005 < delta_dist < 1.0:
                self.total_distance += delta_dist

        self.prev_x = curr_x
        self.prev_y = curr_y

        # Update instantaneous linear speed from odometry
        odom_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if odom_speed > 0.001:
            self.current_speed = odom_speed

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Track commanding velocities to accurately detect motion."""
        cmd_linear = abs(msg.linear.x)
        cmd_angular = abs(msg.angular.z)
        self.current_angular_speed = cmd_angular

        # If odom hasn't registered yet, use command velocity
        if self.current_speed <= 0.001:
            self.current_speed = cmd_linear

        # Robot is moving if linear or angular speed exceeds threshold
        self.is_moving = (
            cmd_linear > self.linear_thresh or cmd_angular > self.angular_thresh
        )

    def estop_callback(self, msg: Bool) -> None:
        """Track E-Stop state."""
        self.estop_active = bool(msg.data)
        if self.estop_active:
            self.is_moving = False
            self.current_speed = 0.0

    def timer_callback(self) -> None:
        """Periodic telemetry calculation, publish, console print, and CSV logging."""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        if dt > 0 and dt < 5.0:
            if self.is_moving and not self.estop_active:
                self.total_motion_time += dt
                # Drain 1% every seconds_per_pct of motion
                drain_amount = (dt / self.seconds_per_pct) * 1.0
                self.battery_percent = max(0.0, self.battery_percent - drain_amount)

        # Average speed: total distance / total motion time (if motion has occurred)
        avg_speed = (
            (self.total_distance / self.total_motion_time)
            if self.total_motion_time > 0.5
            else self.current_speed
        )

        sim_time_sec = now.nanoseconds / 1e9

        # 1. Publish JSON Telemetry Message
        telemetry_data = {
            'timestamp': round(sim_time_sec, 2),
            'distance_travelled_m': round(self.total_distance, 3),
            'current_speed_mps': round(self.current_speed, 3),
            'avg_speed_mps': round(avg_speed, 3),
            'battery_percent': round(self.battery_percent, 1),
            'motion_time_sec': round(self.total_motion_time, 1),
            'is_moving': self.is_moving,
            'estop_active': self.estop_active
        }
        msg = String()
        msg.data = json.dumps(telemetry_data)
        self.telemetry_pub.publish(msg)

        # 2. Console Telemetry Output (throttled every 2s)
        console_dt = (now - self.last_console_time).nanoseconds / 1e9
        if console_dt >= 2.0:
            self.last_console_time = now
            estop_str = 'HALTED' if self.estop_active else 'ACTIVE'
            self.get_logger().info(
                f'[TELEMETRY] Dist: {self.total_distance:.2f} m | '
                f'Speed: {self.current_speed:.2f} m/s (Avg: {avg_speed:.2f} m/s) | '
                f'Battery: {self.battery_percent:.1f}% | State: {estop_str}'
            )

        # 3. Write to CSV Log
        try:
            with open(self.csv_log_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f'{sim_time_sec:.2f}',
                    f'{self.total_distance:.3f}',
                    f'{self.current_speed:.3f}',
                    f'{avg_speed:.3f}',
                    f'{self.battery_percent:.1f}',
                    f'{self.total_motion_time:.1f}',
                    self.is_moving,
                    self.estop_active
                ])
        except Exception as e:
            self.get_logger().warn(f'Error writing telemetry to CSV: {e}', throttle_duration_sec=5.0)


def main(args=None) -> None:
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
