import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16
from datetime import datetime
from modules.motors.OsoyooBase import OsoyooBase
from modules.utils.func_utils import load_configuration
from myrobotics_protocol.msg import BaseInfos, Velocity

class OsoyooBaseController(Node):
    def __init__(self):
        super().__init__('osoyoo_base')
        configuration = load_configuration('/home/pi/.neutron/osoyoo_base.json')
        self.robot = OsoyooBase(configuration)
        self.stream_infos = self.create_publisher(String, 'topic', 10)

        # self.__keep_alive_timeout__ = 0.5
        # self.__keep_alive_time__ = 0
        # keep_alive_timer_period = 0.5  # seconds        

        self.moveSubscription = self.create_subscription(
            Velocity,
            'set_velocity',
            self.set_velocity,
            10)
        self.stopSubscription = self.create_subscription(
            String,
            'stop',
            self.stop,
            1)
        # self.keepAliveSubscription = self.create_subscription(
        #     String,
        #     'keep_alive',
        #     self.keep_alive,
        #     10)

        self.postInfosPublisher = self.create_publisher(
            BaseInfos,
            'infos',
            10)

        # self.keep_alive_timer = self.create_timer(
        #     keep_alive_timer_period, self.keep_alive_timer_callback
        #     )
        self.postInfosTimer = self.create_timer(
            1, self.post_infos_callback
            )

    def set_velocity(self, msg):
        """Apply the transformation received to the robot to move it"""
        print("receive velocity" + str(msg.x) + str(msg.yaw), flush=True)
        self.robot.set_velocity(msg.x, msg.yaw)
        # self.__keep_alive_time__ = datetime.now()

    def stop(self, msg):
        """Stop the robot"""
        print("Stopping")
        if (self.robot.is_moving):
            self.robot.stop()
        else:
            print("The robot is already stopped")

    # def keep_alive(self, msg):
    #     """Keep the robot alive"""
    #     self.__keep_alive_time__ = datetime.now()
    
    def post_infos_callback(self):
        """Publish paramter from the robot"""
        msg = BaseInfos()
        msg.name = "Osoyoo base"
        msg.is_moving = self.robot.is_moving
        msg.speed = self.robot.speed
        msg.max_speed = self.robot.max_speed
        self.postInfosPublisher.publish(msg)

    # def keep_alive_timer_callback(self):
    #     if (not self.robot.is_moving):
    #         return

    #     now = datetime.now()
    #     if (now - self.__keep_alive_time__).total_seconds() > self.__keep_alive_timeout__:
    #         print("Timeout stopped the robot")
    #         self.robot.stop()

def main(args=None):
    rclpy.init(args=args)

    osoyoo_base = OsoyooBaseController()

    print("Osoyoo base started:")
    rclpy.spin(osoyoo_base)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    osoyoo_base.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()