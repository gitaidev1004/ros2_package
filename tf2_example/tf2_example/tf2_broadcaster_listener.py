import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from rclpy.duration import Duration

class Tf2BroadcasterListener(Node):
    def __init__(self):
        super().__init__('tf2_broadcaster_listener')

        # TF2 브로드캐스터 생성
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # TF2 리스너 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 타이머 설정: 1초마다 브로드캐스트 및 리스닝
        self.timer = self.create_timer(1.0, self.broadcast_and_listen)

        # 자이로센서/로봇 프레임을 브로드캐스트
        self.frame_id = 'base_link'
        self.child_frame_id = 'camera_link'

    def broadcast_and_listen(self):
        # TF2 브로드캐스팅
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        # Transform 설정 (예시로 카메라가 'base_link'에서 1.0m 이동한 경우)
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # TF를 브로드캐스트
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Broadcasting transform from {self.frame_id} to {self.child_frame_id}')

        # 리스너로 특정 프레임의 위치 추적
        try:
            transform = self.tf_buffer.lookup_transform(self.frame_id, self.child_frame_id, rclpy.time.Time())
            self.get_logger().info(f'Received transform: {transform}')
        except (tf2_ros.TransformException) as e:
            self.get_logger().info(f'Could not get transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Tf2BroadcasterListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

