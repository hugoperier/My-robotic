import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
from modules.motors.OsoyooBase import OsoyooBase
from myrobotics_protocol.msg import BaseInfos

class OsoyooBaseController(Node):
    def __init__(self):
        super().__init__('osoyoo_base')
        self.robot = OsoyooBase()
        self.stream_infos = self.create_publisher(String, 'topic', 10)

        self.__keep_alive_timeout__ = 0.5
        self.__keep_alive_time__ = 0
        keep_alive_timer_period = 0.5  # seconds        

        self.moveSubscription = self.create_subscription(
            String,
            'move',
            self.move,
            10)
        self.stopSubscription = self.create_subscription(
            String,
            'stop',
            self.stop,
            10)
        self.keepAliveSubscription = self.create_subscription(
            String,
            'keep_alive',
            self.keep_alive,
            10)
        self.setSpeedSubscription = self.create_subscription(
            int16,
            'set_speed',
            self.listener_callback,
            10)

        self.postInfosPublisher = self.create_publisher(
            String,
            'infos',
            10)

        self.keep_alive_timer = self.create_timer(
            keep_alive_timer_period, self.keep_alive_timer_callback
            )
        self.postInfosTimer = self.create_timer(
            1, self.post_infos_timer_callback
            )

    def move(self, msg):
        """Move the robot"""
        if (msg.data == "forward"):
            self.robot.forward()
        elif (msg.data == "backward"):
            self.robot.backward()
        elif (msg.data == "left"):
            self.robot.left()
        elif (msg.data == "right"):
            self.robot.right()
        else:
            print("Unknown move command")
        return response

    def stop(self, msg):
        """Stop the robot"""
        if (self.robot.is_moving):
            self.robot.stop()
        else:
            print("The robot is already stopped")

    def keep_alive(self):
        """Keep the robot alive"""
        self.__keep_alive_time__ = datetime.now()
        return response
    
    def post_infos_callback(self):
        """Publish paramter from the robot"""
        msg = BaseInfos()
        msg.name = self.robot.name
        msg.wheel_count = len(self.robot.wheels)
        msg.is_moving = self.robot.is_moving
        msg.speed = self.robot.speed
        msg.max_speed = self.robot.max_speed
        self.postInfosPublisher.publish(msg)

    def keep_alive_timer_callback(self):
        if (not self.robot.is_moving):
            return

        now = datetime.now()
        if (now - self.__keep_alive_time__).total_seconds() > self.__keep_alive_timeout__:
            print("Timeout stopped the robot")
            self.stop()

def main(args=None):
    rclpy.init(args=args)

    osoyoo_base = OsoyooBaseController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()