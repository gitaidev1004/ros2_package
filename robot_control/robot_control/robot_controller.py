import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Robot Controller Node has started.")
        
    def timer_callback(self):
        msg = Twist()
        
        # 속도 설정
        msg.linear.x = 0.5  # 전진 속도 (m/s)
        msg.angular.z = 0.1  # 회전 속도 (rad/s)

        # 로봇에 메시지 퍼블리시
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing cmd_vel: Linear: %.2f, Angular: %.2f' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
