import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Countdown

class CountdownActionClient(Node):
    def __init__(self):
        super().__init__('countdown_action_client')
        self.client = ActionClient(self, Countdown, 'countdown')

    def send_goal(self, start_number):
        goal_msg = Countdown.Goal()
        goal_msg.start_number = start_number

        self.client.wait_for_server()
        self.get_logger().info(f'Sending goal: Start countdown from {start_number}')
        
        future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.current_number}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Countdown complete! Progress: {result.progress}')
        rclpy.shutdown()

def main():
    rclpy.init()
    client = CountdownActionClient()
    client.send_goal(5)
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()