import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray


class PatrolController(Node):
    """
    Autonomous Patrol and Obstacle Avoidance Controller for TurtleBot3 Burger.

    Finite-State Machine:
      - PATROL: Navigate through configured waypoints using odometry.
      - AVOID: Rotate away from detected obstacles towards the clearest sector.
      - CLEAR: Cautiously move forward past the obstacle with side repulsion.
      - RECOVERY: Bounded recovery maneuvers if progress is stalled.
      - BLOCKED: Safe stationary state if no path is found after max retries.
      - HALTED: Immediate safety stop when emergency stop service is triggered.
    """

    def __init__(self):
        super().__init__('patrol_controller')

        # Declare configurable parameters
        self.declare_parameter('obstacle_distance_threshold', 0.38)
        self.declare_parameter('safe_resume_distance', 0.48)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.28)
        self.declare_parameter('square_size', 0.8)
        self.declare_parameter('waypoint_tolerance', 0.18)
        self.declare_parameter('stuck_timeout', 3.0)
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('scan_timeout', 1.5)
        self.declare_parameter('odom_timeout', 1.5)
        self.declare_parameter(
            'waypoints',
            [-2.0, -0.5, -1.4, -0.5, -1.4, 0.5, -2.0, 0.5]
        )

        # Retrieve parameters
        self.obs_thresh = float(
            self.get_parameter('obstacle_distance_threshold').value)
        self.safe_resume_dist = float(
            self.get_parameter('safe_resume_distance').value)
        self.linear_speed = float(
            self.get_parameter('linear_speed').value)
        self.angular_speed = float(
            self.get_parameter('angular_speed').value)
        self.square_size = float(
            self.get_parameter('square_size').value)
        self.wp_tolerance = float(
            self.get_parameter('waypoint_tolerance').value)
        self.stuck_timeout = float(
            self.get_parameter('stuck_timeout').value)
        self.max_recovery_attempts = int(
            self.get_parameter('max_recovery_attempts').value)
        self.scan_timeout = float(
            self.get_parameter('scan_timeout').value)
        self.odom_timeout = float(
            self.get_parameter('odom_timeout').value)

        # Parse waypoints parameter
        raw_wps = self.get_parameter('waypoints').value
        self.configured_waypoints: List[Tuple[float, float]] = []
        if raw_wps and len(raw_wps) >= 4 and len(raw_wps) % 2 == 0:
            for i in range(0, len(raw_wps), 2):
                self.configured_waypoints.append(
                    (float(raw_wps[i]), float(raw_wps[i + 1]))
                )

        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Float32MultiArray, '/obstacle_status', self.obstacle_callback, 10
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.create_subscription(
            Bool, '/estop_state', self.estop_callback, 10
        )

        # Perception state
        self.front = 3.5
        self.left = 3.5
        self.right = 3.5
        self.have_scan = False
        self.last_scan_time = None

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = None
        self.origin = None
        self.waypoints: List[Tuple[float, float]] = []
        self.waypoint_index = 0

        # Safety / E-stop state
        self.estop = False

        # State Machine: 'PATROL', 'AVOID', 'CLEAR', 'RECOVERY', 'BLOCKED', 'HALTED'
        self.state = 'PATROL'
        self.turn_direction = 1.0  # +1.0 = left (CCW), -1.0 = right (CW)
        self.avoid_start_time = 0.0
        self.clear_start_time = 0.0
        self.recovery_attempts = 0
        self.recovery_start_time = 0.0
        self.recovery_phase = 0

        # Progress / Stuck detection
        self.progress_anchor = None
        self.progress_start_time = None
        self.last_cmd_linear = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Patrol Controller initialized (obstacle_threshold={self.obs_thresh:.2f}m, '
            f'safe_resume={self.safe_resume_dist:.2f}m, linear_speed={self.linear_speed:.2f}m/s)'
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def obstacle_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            f, l, r = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
            if all(math.isfinite(v) and v >= 0.0 for v in (f, l, r)):
                self.front = f
                self.left = l
                self.right = r
                self.have_scan = True
                self.last_scan_time = self.now_sec()

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.x = pos.x
        self.y = pos.y

        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.last_odom_time = self.now_sec()

        if self.origin is None:
            self.origin = (self.x, self.y, self.yaw)
            sx, sy, syaw = self.origin

            if self.configured_waypoints:
                self.waypoints = list(self.configured_waypoints)
                wp_str = ', '.join(f'({wx:.2f}, {wy:.2f})' for wx, wy in self.waypoints)
                self.get_logger().info(f'Using configured route waypoints: {wp_str}')
            else:
                c = math.cos(syaw)
                s = math.sin(syaw)
                L = self.square_size
                rel_wps = [(L, 0.0), (L, L), (0.0, L), (0.0, 0.0)]
                self.waypoints = [
                    (sx + dx * c - dy * s, sy + dx * s + dy * c)
                    for dx, dy in rel_wps
                ]
                wp_str = ', '.join(f'({wx:.2f}, {wy:.2f})' for wx, wy in self.waypoints)
                self.get_logger().info(
                    f'Relative square route generated ({L:.2f}m) around '
                    f'origin ({sx:.2f}, {sy:.2f}): {wp_str}'
                )

    def estop_callback(self, msg: Bool):
        prev = self.estop
        self.estop = bool(msg.data)
        if self.estop and not prev:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED — Robot stopping immediately')
            self.publish_cmd(0.0, 0.0)
        elif not self.estop and prev:
            self.get_logger().info('EMERGENCY STOP RELEASED — Robot resuming operations')

    def publish_cmd(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)
        self.last_cmd_linear = float(linear)

    def is_stuck(self, now: float) -> bool:
        if abs(self.last_cmd_linear) < 0.04:
            self.progress_anchor = None
            self.progress_start_time = None
            return False

        if self.progress_anchor is None:
            self.progress_anchor = (self.x, self.y)
            self.progress_start_time = now
            return False

        dist_moved = math.hypot(
            self.x - self.progress_anchor[0],
            self.y - self.progress_anchor[1]
        )
        if dist_moved >= 0.04:
            self.progress_anchor = (self.x, self.y)
            self.progress_start_time = now
            return False

        if now - self.progress_start_time > self.stuck_timeout:
            self.progress_anchor = None
            self.progress_start_time = None
            return True

        return False

    def enter_avoidance(self, now: float, reason: str):
        self.state = 'AVOID'
        self.avoid_start_time = now

        # Turn toward the side with greater measured clearance
        if self.left > self.right + 0.05:
            self.turn_direction = 1.0  # Turn Left
        elif self.right > self.left + 0.05:
            self.turn_direction = -1.0  # Turn Right
        else:
            self.turn_direction = 1.0

        turn_side = "LEFT" if self.turn_direction > 0 else "RIGHT"
        self.get_logger().warn(
            f'State -> AVOID ({reason}) | Front={self.front:.2f}m (<{self.obs_thresh:.2f}m), '
            f'Left={self.left:.2f}m, Right={self.right:.2f}m. Turning {turn_side}'
        )
        self.publish_cmd(0.0, self.angular_speed * self.turn_direction)

    def enter_recovery(self, now: float, reason: str):
        self.recovery_attempts += 1
        if self.recovery_attempts > self.max_recovery_attempts:
            self.state = 'BLOCKED'
            self.publish_cmd(0.0, 0.0)
            self.get_logger().error(
                f'State -> BLOCKED: Maximum recovery attempts ({self.max_recovery_attempts}) '
                f'exceeded. Robot safely halted. ({reason})'
            )
            return

        self.state = 'RECOVERY'
        self.recovery_start_time = now
        self.recovery_phase = 1
        self.get_logger().warn(
            f'State -> RECOVERY (Attempt {self.recovery_attempts}/'
            f'{self.max_recovery_attempts}): {reason}'
        )

    def control_loop(self):
        now = self.now_sec()

        # 1. EMERGENCY STOP CHECK (Highest priority)
        if self.estop:
            self.publish_cmd(0.0, 0.0)
            return

        # 2. SENSOR & ODOMETRY TIMEOUT / STALE DATA CHECK
        scan_stale = (
            not self.have_scan or
            self.last_scan_time is None or
            (now - self.last_scan_time > self.scan_timeout)
        )
        if scan_stale:
            self.publish_cmd(0.0, 0.0)
            return

        odom_stale = (
            self.last_odom_time is None or
            (now - self.last_odom_time > self.odom_timeout) or
            self.origin is None
        )
        if odom_stale:
            self.publish_cmd(0.0, 0.0)
            return

        # 3. STATE MACHINE

        # --- BLOCKED STATE ---
        if self.state == 'BLOCKED':
            self.publish_cmd(0.0, 0.0)
            if self.front >= self.safe_resume_dist:
                self.get_logger().info('Clearance detected while BLOCKED. Resuming PATROL.')
                self.recovery_attempts = 0
                self.state = 'PATROL'
            return

        # --- RECOVERY STATE ---
        if self.state == 'RECOVERY':
            elapsed = now - self.recovery_start_time
            if self.recovery_phase == 1:
                # Phase 1: Reverse slightly for 1.2s
                if elapsed < 1.2:
                    self.publish_cmd(-0.06, 0.0)
                    return
                else:
                    self.recovery_phase = 2
                    self.recovery_start_time = now
                    self.turn_direction *= -1.0
                    return
            elif self.recovery_phase == 2:
                # Phase 2: Rotate away for 1.5s
                if elapsed < 1.5:
                    self.publish_cmd(0.0, self.angular_speed * self.turn_direction)
                    return
                else:
                    if self.front >= self.safe_resume_dist:
                        self.get_logger().info(
                            f'Recovery successful! Front clear ({self.front:.2f}m). '
                            'Resuming PATROL.'
                        )
                        self.recovery_attempts = 0
                        self.state = 'PATROL'
                        self.publish_cmd(0.0, 0.0)
                    else:
                        self.state = 'AVOID'
                        self.avoid_start_time = now
                    return

        # --- AVOID STATE ---
        if self.state == 'AVOID':
            elapsed = now - self.avoid_start_time

            # When front clearance exceeds safe_resume_distance: transition to CLEAR
            if self.front >= self.safe_resume_dist:
                self.state = 'CLEAR'
                self.clear_start_time = now
                self.recovery_attempts = 0
                self.get_logger().info(
                    f'Front clear ({self.front:.2f}m >= {self.safe_resume_dist:.2f}m). '
                    f'Transitioning to CLEAR.'
                )
                self.publish_cmd(0.0, 0.0)
                return

            # Timeout spinning in place without finding clearance: trigger recovery
            if elapsed > 3.5:
                self.enter_recovery(now, 'AVOID rotation timeout with no clear opening')
                return

            self.publish_cmd(0.0, self.angular_speed * self.turn_direction)
            return

        # --- CLEAR STATE (cautiously moving past obstacle) ---
        if self.state == 'CLEAR':
            if self.front < self.obs_thresh:
                self.enter_avoidance(now, 'Obstacle re-encountered during CLEAR')
                return

            elapsed = now - self.clear_start_time
            if elapsed < 1.0:
                steering = 0.0
                if self.left < 0.22:
                    steering = -0.10
                elif self.right < 0.22:
                    steering = 0.10
                self.publish_cmd(0.08, steering)
                return

            self.state = 'PATROL'
            self.get_logger().info('CLEAR phase complete. Resuming PATROL.')
            self.publish_cmd(0.0, 0.0)
            return

        # --- PATROL STATE ---
        if self.front < self.obs_thresh:
            self.enter_avoidance(now, f'Obstacle ahead at {self.front:.2f}m')
            return

        if self.is_stuck(now):
            self.enter_recovery(now, 'Robot progress stalled while driving forward')
            return

        if not self.waypoints:
            self.publish_cmd(0.0, 0.0)
            return

        # Target waypoint
        tx, ty = self.waypoints[self.waypoint_index]
        dx = tx - self.x
        dy = ty - self.y
        dist_to_wp = math.hypot(dx, dy)

        # Waypoint reached check
        if dist_to_wp <= self.wp_tolerance:
            prev_idx = self.waypoint_index
            self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
            next_wp = self.waypoints[self.waypoint_index]
            self.get_logger().info(
                f'Waypoint {prev_idx} reached! Advancing to waypoint {self.waypoint_index} '
                f'({next_wp[0]:.2f}, {next_wp[1]:.2f})'
            )
            self.publish_cmd(0.0, 0.0)
            return

        # Target heading
        target_yaw = math.atan2(dy, dx)
        heading_err = math.atan2(
            math.sin(target_yaw - self.yaw),
            math.cos(target_yaw - self.yaw)
        )

        # Rotate in place only if large heading error (> 0.45 rad ≈ 26°)
        if abs(heading_err) > 0.45:
            turn_cmd = self.angular_speed if heading_err > 0.0 else -self.angular_speed
            self.publish_cmd(0.0, turn_cmd)
            return

        # Heading aligned: drive forward with smooth proportional steering
        linear_cmd = min(self.linear_speed, max(0.05, dist_to_wp))
        angular_cmd = max(-self.angular_speed, min(self.angular_speed, 0.9 * heading_err))

        # Gentle side obstacle repulsion
        if self.left < 0.22:
            angular_cmd -= 0.10
        elif self.right < 0.22:
            angular_cmd += 0.10

        self.publish_cmd(linear_cmd, angular_cmd)


def main(args=None):
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
