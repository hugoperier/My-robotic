import rclpy
from myrobotics_protocol.srv import GlobalResult
from rclpy.node import Node
from std_msgs.msg import String, Int16
from myrobotics_protocol.msg import Velocity, BaseInfos

class TestController(Node):
    def __init__(self):
        super().__init__('testnode')
        self.__keep_alive_timeout__ = 0.5
        self.__keep_alive_time__ = 0
        keep_alive_timer_period = 5  # seconds 
        self.testSubscription = self.create_subscription(
            Velocity,
            'test_topic',
            self.test_subscription,
            10)
        self.testPublisher = self.create_publisher(
            BaseInfos,
            'test_pub',
            1)
        self.testService = self.create_service(
            GlobalResult,
            'test_srv',
            self.test_srv)
        self.keep_alive_timer = self.create_timer(
            keep_alive_timer_period, self.test_publisher
            )

    def test_publisher(self):
        msg = BaseInfos()
        msg.name = "Hello pubs"
        msg.wheel_count = 4
        msg.is_moving = True
        msg.max_speed = 100
        msg.speed = 30
        self.testPublisher.publish(msg)
        print("publishing")

    def test_subscription(self, msg):
        print("received test subsctription " + str(msg.x) + ";" + str(msg.y))

    def test_srv(self, request, response):
        print("req")
        print(request)
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    print('Hi from testnode haha.')
    testNode = TestController()
    rclpy.spin(testNode)
    print("goodbye")
    testNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
