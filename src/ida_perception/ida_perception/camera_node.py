import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Image publisher
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)

        # OpenCV kamera açma
        self.cap = cv2.VideoCapture(0)  # 0 = laptop kamerası (YANİ WEBCAM)

        if not self.cap.isOpened():
            self.get_logger().error("Kamera açılamadı!")
            return

        self.bridge = CvBridge()

        # 30 FPS için timer
        self.timer = self.create_timer(1/30, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Kamera görüntüsü alınamadı!")
            return

        # OpenCV (BGR) → ROS Image mesajına çevir
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # Yayınla
        self.publisher_.publish(img_msg)
        self.get_logger().info("Kamera görüntüsü yayınlandı")

    def destroy_node(self):
        # Kamera kapat
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
