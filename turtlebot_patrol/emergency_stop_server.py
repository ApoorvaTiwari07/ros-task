#!/usr/bin/env python3
"""
emergency_stop_server.py

Safety service node for TurtleBot3.
- Provides service /trigger_estop (std_srvs/srv/SetBool).
  * true  -> Activates emergency-stop.
  * false -> Clears emergency-stop.
- Publishes the current state on /estop_state (std_msgs/msg/Bool).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class EmergencyStopServer(Node):
    """Emergency stop safety service and publisher node."""

    def __init__(self) -> None:
        super().__init__('emergency_stop_server')

        # Parameters
        self.declare_parameter('initial_state', False)
        self.declare_parameter('publish_rate', 10.0)

        self.estop_active = bool(self.get_parameter('initial_state').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # Service /trigger_estop using SetBool
        self.srv_set_bool = self.create_service(
            SetBool, '/trigger_estop', self.handle_trigger_estop
        )

        # Optional Trigger fallback service /trigger_estop_toggle for convenience
        self.srv_trigger = self.create_service(
            Trigger, '/trigger_estop_toggle', self.handle_trigger_toggle
        )

        # Publisher for /estop_state
        self.estop_pub = self.create_publisher(Bool, '/estop_state', 10)

        # Timer to regularly broadcast estop state
        timer_period = 1.0 / max(publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_state)

        self.get_logger().info(
            f'EmergencyStopServer active. Initial state: {self.estop_active}'
        )

    def publish_state(self) -> None:
        """Publish current E-stop state to /estop_state."""
        msg = Bool()
        msg.data = self.estop_active
        self.estop_pub.publish(msg)

    def handle_trigger_estop(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Handle SetBool requests: true activates, false clears."""
        self.estop_active = bool(request.data)
        self.publish_state()

        if self.estop_active:
            response.success = True
            response.message = 'EMERGENCY STOP ACTIVATED: Robot halted immediately.'
            self.get_logger().warn(response.message)
        else:
            response.success = True
            response.message = 'EMERGENCY STOP CLEARED: Robot resumed normal patrol.'
            self.get_logger().info(response.message)

        return response

    def handle_trigger_toggle(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Toggle emergency stop state via Trigger interface."""
        self.estop_active = not self.estop_active
        self.publish_state()

        state_str = 'ACTIVATED' if self.estop_active else 'CLEARED'
        response.success = True
        response.message = f'Emergency stop toggled to {state_str}.'
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmergencyStopServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
