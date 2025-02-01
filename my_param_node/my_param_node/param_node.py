import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')

        # 초기 파라미터 설정
        self.declare_parameter('my_param', 'default_value')

        # 타이머를 사용하여 파라미터 값을 출력
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 현재 파라미터 값 가져오기
        param_value = self.get_parameter('my_param').get_parameter_value().string_value
        self.get_logger().info(f'Current parameter value: {param_value}')

def main(args=None):
    rclpy.init(args=args)

    node = ParamNode()

    # 파라미터 변경 예시 (node 내부에서 동적으로 값을 변경할 수 있음)
    node.set_parameters([Parameter('my_param', Parameter.Type.STRING, 'new_value')])

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()