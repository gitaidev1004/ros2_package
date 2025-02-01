import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 카메라 이미지 토픽
            self.listener_callback,
            10)
        self.get_logger().info('Image Listener Node has started.')

    def listener_callback(self, msg):
        # ROS 메시지를 OpenCV 이미지로 변환
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # OpenCV를 사용하여 이미지 처리 (예: 이미지 반전)
            cv_image = cv2.flip(cv_image, 1)  # 이미지를 수평으로 반전
            # 이미지 출력
            cv2.imshow("Processed Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
