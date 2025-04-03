import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Assurez-vous que ce topic correspond à celui de votre caméra
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            decoded_objects = pyzbar.decode(cv_image)
            for obj in decoded_objects:
                self.get_logger().info(f"QR Code detected: {obj.data.decode('utf-8')}")
                # Vous pouvez également publier les données du QR code sur un topic si nécessaire
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    qr_code_detector = QRCodeDetector()
    rclpy.spin(qr_code_detector)
    qr_code_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()