import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureListener(Node):
    def __init__(self):
        super().__init__('temperature_listener')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received temperature: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
