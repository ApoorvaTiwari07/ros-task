#!/usr/bin/env python3
"""
patrol_controller.py

Decision Making & Control node for TurtleBot3.
- Subscribes to /obstacle_status, /odom, and /estop_state.
- Publishes velocity commands to /cmd_vel.
- State Machine:
  * HALTED: When emergency stop is active, publishes zero linear and angular velocity.
  * AVOID: When an obstacle is detected in front (< threshold), stops forward motion
           and rotates toward the more open side (left vs right) until clear.
  * PATROL: Follows a square route or configured waypoints using odometry feedback.
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class PatrolController(Node):
    """Finite State Machine controller for TurtleBot3 autonomous patrol."""

    # FSM States
    STATE_PATROL = 'PATROL'
    STATE_AVOID = 'AVOID'
    STATE_HALTED = 'HALTED'

    def __init__(self) -> None:
        super().__init__('patrol_controller')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('obstacle_threshold', 0.50)
        self.declare_parameter('clear_threshold', 0.65)
        self.declare_parameter('avoid_angular_speed', 0.70)
        self.declare_parameter('square_size', 2.0)
        self.declare_parameter('waypoint_tolerance', 0.20)
        self.declare_parameter('heading_tolerance', 0.25)
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter(
            'waypoints',
            [0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0]
        )

        # Retrieve parameters
        self.max_linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('angular_speed').value)
        self.obstacle_threshold = float(self.get_parameter('obstacle_threshold').value)
        self.clear_threshold = float(self.get_parameter('clear_threshold').value)
        self.avoid_angular_speed = float(self.get_parameter('avoid_angular_speed').value)
        self.square_size = float(self.get_parameter('square_size').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.heading_tolerance = float(self.get_parameter('heading_tolerance').value)
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        control_rate = float(self.get_parameter('control_rate').value)

        # Parse waypoints
        raw_waypoints = self.get_parameter('waypoints').value
        self.waypoints: List[Tuple[float, float]] = []
        if raw_waypoints and len(raw_waypoints) >= 4 and len(raw_waypoints) % 2 == 0:
            for i in range(0, len(raw_waypoints), 2):
                self.waypoints.append((float(raw_waypoints[i]), float(raw_waypoints[i + 1])))
        else:
            s = self.square_size
            self.waypoints = [(0.0, 0.0), (s, 0.0), (s, s), (0.0, s)]

        self.current_waypoint_idx = 0

        # State tracking
        self.state = self.STATE_PATROL
        self.estop_active = False
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        # Robot Odometry Pose
        self.odom_received = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Avoidance turn direction: +1.0 for left (CCW), -1.0 for right (CW)
        self.avoid_turn_dir = 1.0

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # Subscribers
        self.create_subscription(
            Float32MultiArray, '/obstacle_status', self.obstacle_callback, 10
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.create_subscription(
            Bool, '/estop_state', self.estop_callback, 10
        )

        # Control Loop Timer
        self.timer = self.create_timer(1.0 / max(control_rate, 1.0), self.control_loop)

        self.get_logger().info(
            f'PatrolController started with {len(self.waypoints)} waypoints: {self.waypoints}'
        )

    def obstacle_callback(self, msg: Float32MultiArray) -> None:
        """Receive sector distances from LiDAR obstacle detector."""
        if len(msg.data) >= 3:
            self.front_dist = float(msg.data[0])
            self.left_dist = float(msg.data[1])
            self.right_dist = float(msg.data[2])

    def odom_callback(self, msg: Odometry) -> None:
        """Update robot pose and yaw orientation from odometry."""
        self.odom_received = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Quaternion to yaw conversion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def estop_callback(self, msg: Bool) -> None:
        """Update emergency-stop status."""
        prev = self.estop_active
        self.estop_active = bool(msg.data)
        if prev != self.estop_active:
            if self.estop_active:
                self.get_logger().warn('E-Stop ENGAGED: Halting robot immediately.')
            else:
                self.get_logger().info('E-Stop RELEASED: Resuming controller.')

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        """Publish velocity command to /cmd_vel."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, +pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def control_loop(self) -> None:
        """Main control loop executing the Finite State Machine."""
        # 1. HALTED State Check (Emergency Stop)
        if self.estop_active:
            self.state = self.STATE_HALTED
            self.publish_cmd(0.0, 0.0)
            return

        # 2. Obstacle Detection & Transition to AVOID
        if self.state == self.STATE_PATROL:
            if self.front_dist < self.obstacle_threshold:
                self.state = self.STATE_AVOID
                # Choose the more open side to turn toward
                if self.left_dist >= self.right_dist:
                    self.avoid_turn_dir = 1.0   # Rotate left (CCW)
                else:
                    self.avoid_turn_dir = -1.0  # Rotate right (CW)
                self.get_logger().warn(
                    f'Obstacle detected at {self.front_dist:.2f}m! '
                    f'Transitioning to AVOID. Left: {self.left_dist:.2f}m, Right: {self.right_dist:.2f}m. '
                    f'Turning {"LEFT" if self.avoid_turn_dir > 0 else "RIGHT"}.'
                )

        # 3. State Execution
        if self.state == self.STATE_AVOID:
            # Check if front path has cleared with hysteresis margin
            if self.front_dist >= self.clear_threshold:
                self.state = self.STATE_PATROL
                self.get_logger().info(
                    f'Path clear (front: {self.front_dist:.2f}m >= {self.clear_threshold:.2f}m). Resuming PATROL.'
                )
            else:
                # Rotate in place toward the open direction
                angular_cmd = self.avoid_turn_dir * self.avoid_angular_speed
                self.publish_cmd(linear_x=0.0, angular_z=angular_cmd)
                return

        # 4. PATROL State Execution
        if self.state == self.STATE_PATROL:
            if not self.odom_received:
                self.publish_cmd(0.0, 0.0)
                return

            tx, ty = self.waypoints[self.current_waypoint_idx]
            dx = tx - self.x
            dy = ty - self.y
            dist_to_wp = math.hypot(dx, dy)

            # Check if waypoint reached
            if dist_to_wp <= self.waypoint_tolerance:
                self.get_logger().info(
                    f'Reached Waypoint {self.current_waypoint_idx} at ({tx:.2f}, {ty:.2f})'
                )
                self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
                tx, ty = self.waypoints[self.current_waypoint_idx]
                dx = tx - self.x
                dy = ty - self.y
                dist_to_wp = math.hypot(dx, dy)

            # Heading error calculation
            target_heading = math.atan2(dy, dx)
            heading_err = self.normalize_angle(target_heading - self.yaw)

            # Control logic: turn in place if large heading error, else drive forward
            if abs(heading_err) > self.heading_tolerance:
                # In-place alignment
                angular_z = math.copysign(
                    min(self.max_angular_speed, max(0.2, self.kp_angular * abs(heading_err))),
                    heading_err
                )
                self.publish_cmd(linear_x=0.0, angular_z=angular_z)
            else:
                # Proportional driving with course correction
                linear_x = min(self.max_linear_speed, max(0.05, self.kp_linear * dist_to_wp))
                angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, self.kp_angular * heading_err))
                self.publish_cmd(linear_x=linear_x, angular_z=angular_z)

            # Publish waypoint markers for RViz visualization
            self.publish_waypoint_markers()

    def publish_waypoint_markers(self) -> None:
        """Publish visual marker array of patrol waypoints for RViz."""
        marker_array = MarkerArray()

        # Marker for waypoint path lines
        line_strip = Marker()
        line_strip.header.frame_id = 'odom'
        line_strip.header.stamp = self.get_clock().now().to_msg()
        line_strip.ns = 'patrol_route'
        line_strip.id = 0
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.scale.x = 0.05
        line_strip.color.r = 0.2
        line_strip.color.g = 0.6
        line_strip.color.b = 1.0
        line_strip.color.a = 0.8

        for wp in self.waypoints:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = 0.02
            line_strip.points.append(p)
        # Close loop back to first waypoint
        if self.waypoints:
            p = Point()
            p.x = float(self.waypoints[0][0])
            p.y = float(self.waypoints[0][1])
            p.z = 0.02
            line_strip.points.append(p)

        marker_array.markers.append(line_strip)

        # Marker for current target waypoint sphere
        if self.waypoints:
            target_wp = self.waypoints[self.current_waypoint_idx]
            target_marker = Marker()
            target_marker.header.frame_id = 'odom'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'target_waypoint'
            target_marker.id = 1
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = float(target_wp[0])
            target_marker.pose.position.y = float(target_wp[1])
            target_marker.pose.position.z = 0.1
            target_marker.pose.orientation.w = 1.0
            target_marker.scale.x = 0.25
            target_marker.scale.y = 0.25
            target_marker.scale.z = 0.25
            target_marker.color.r = 1.0
            target_marker.color.g = 0.8
            target_marker.color.b = 0.0
            target_marker.color.a = 0.95
            marker_array.markers.append(target_marker)

        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
