import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class EmergencyStopServer(Node):
    """
    Emergency Stop Service and State Server for TurtleBot3.

    Provides /trigger_estop service (std_srvs/srv/SetBool) and publishes
    continuous state on /estop_state (std_msgs/msg/Bool).
    """

    def __init__(self):
        super().__init__('emergency_stop_server')

        # Declare configurable parameters
        self.declare_parameter('publish_rate', 10.0)
        rate = self.get_parameter('publish_rate').value

        self.stopped = False

        self.publisher = self.create_publisher(
            Bool, '/estop_state', 10
        )
        self.service = self.create_service(
            SetBool, '/trigger_estop', self.handle_estop
        )
        self.timer = self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().info('Emergency Stop Service ready on /trigger_estop')

    def publish_state(self):
        msg = Bool()
        msg.data = self.stopped
        self.publisher.publish(msg)

    def handle_estop(self, request, response):
        self.stopped = bool(request.data)
        response.success = True
        if self.stopped:
            response.message = 'Emergency stop ACTIVE'
            self.get_logger().warn('Emergency stop triggered: STOPPING ROBOT')
        else:
            response.message = 'Emergency stop RELEASED'
            self.get_logger().info('Emergency stop released: RESUMING OPERATION')

        self.publish_state()
        return response


def main(args=None):
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
