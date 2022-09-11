import rclpy
from rclpy.node import Node
from modules.camera.CameraImpl import CameraImpl
from modules.utils.func_utils import load_configuration
from myrobotics_protocol.srv import GlobalResult, GlobalSetValue, CameraGetInfos

class CameraService(Node):

    def __init__(self):
        super().__init__('camera')
        config = load_configuration("/home/hugoperier/.myrobotics/camera_raspberry.json")
        self.camera = CameraImpl(config)

        self.camera_set_fps = self.create_service(GlobalSetValue, 'set_fps', self.set_fps)
        self.camera_set_resolution = self.create_service(GlobalSetValue, 'set_resolution', self.set_resolution)
        self.camera_start_service = self.create_service(GlobalResult, 'start_camera', self.start_camera)
        self.camera_stop_service = self.create_service(GlobalResult, 'stop_camera', self.stop_camera)
        self.camera_get_infos_service = self.create_service(CameraGetInfos, "getInfos", self.get_infos)


    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

    def set_fps(self, request, response):
        """Set parameter of the camera (width, height, fps)"""

        self.camera.fps = request.value
        return response
    
    def set_resolution(self, request, response):
        success = True
        try:
            self.camera.set_resolution(request.value)
        except:
            success = False
        response.success = success
        return response

    def start_camera(self, request, response):
        """Start the camera server on the port defined in the configuration file"""
        success = True
        try:
            self.camera.make_server()
        except:
            success = False
        response.success = success
        return response


    def stop_camera(self, request, response):
        """Stop the camera server"""
        success = True
        try:
            self.camera.stop_server()
        except:
            success = False
        response.success = success
        return response
    
    def get_infos(self, request, response):
        """Get paramter from the camera"""

        response.resolution = self.camera.resolution
        response.fps = self.camera.fps
        response.on = self.camera.is_streaming
        return response

def main(args=None):
    rclpy.init(args=args)

    camera_service = CameraService()

    rclpy.spin(camera_service)    

    rclpy.shutdown()


if __name__ == '__main__':
    main()