import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action import Countdown
from rclpy.executors import MultiThreadedExecutor
import time

class CountdownActionServer(Node):
    def __init__(self):
        super().__init__('countdown_action_server')
        self.action_server = ActionServer(
            self,
            Countdown,
            'countdown',
            self.execute_callback
        )
        self.get_logger().info('Countdown Action Server is Ready.')

    async def execute_callback(self, goal_handle):
        start_number = goal_handle.request.start_number
        self.get_logger().info(f'Starting countdown from {start_number}')

        for i in range(start_number, -1, -1):
            self.get_logger().info(f'Counting: {i}')
            goal_handle.publish_feedback(Countdown.Feedback(current_number=i))
            time.sleep(1)

        goal_handle.succeed()
        result = Countdown.Result()
        result.progress = 100
        return result

def main():
    rclpy.init()
    node = CountdownActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()