import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 호출
        self.get_logger().info("Temperature Publisher Node has started.")

    def timer_callback(self):
        # 랜덤 온도 값 생성 (-10 ~ 40도 범위)
        temperature = random.uniform(-10.0, 40.0)
        msg = Float32()
        msg.data = temperature
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {temperature:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
