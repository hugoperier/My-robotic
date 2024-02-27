import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        self.start_publisher = self.create_publisher(String, 'start_command', 10)
        self.stop_publisher = self.create_publisher(String, 'stop_command', 10)

        self.timer_period_ = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period_, self.publish_command)

    def publish_command(self):
        user_input = input("Enter 'start' to start the camera or 'stop' to stop it: ")
        msg = String()
        if user_input.lower() == 'start':
            self.start_publisher.publish(msg)
        elif user_input.lower() == 'stop':
            self.stop_publisher.publish(msg)
        else:
            print("Invalid input. Please enter 'start' or 'stop'.")
            return
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    camera_controller = CameraController()
    rclpy.spin(camera_controller)
    camera_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()