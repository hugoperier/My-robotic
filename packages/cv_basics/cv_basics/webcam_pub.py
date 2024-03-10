import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
 
class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'video_frames', 1)
        self.timer = None
        self.cap = None
        self.br = CvBridge()
        self.is_publishing = False

        print("Camera started!")

        self.create_subscription(String, 'start_command', self.start_callback, 10)
        self.create_subscription(String, 'stop_command', self.stop_callback, 10)

    def start_callback(self, msg):
        if not self.is_publishing:
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.cap = cv2.VideoCapture('/dev/video10')
            self.is_publishing = True
            self.get_logger().info('Started publishing video frames')

    def stop_callback(self, msg):
        if self.is_publishing:
            if self.timer:
                self.timer.cancel()
            if self.cap:
                self.cap.release()
            self.is_publishing = False
            self.get_logger().info('Stopped publishing video frames')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg2 = self.br.cv2_to_compressed_imgmsg(frame)
            self.get_logger().info('Sending frame!')
            self.publisher_.publish(msg2)
        else:
            self.get_logger().warning('Failed to capture frame')
  
def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
