import rclpy
from rclpy.node import Node
from modules.camera.CameraImpl import CameraImpl
from modules.utils.func_utils import load_configuration
from myrobotics_protocol.srv import GlobalResult, GlobalSetValue
from myrobotics_protocol.msg import CameraInfo, CameraResolution

class CameraService(Node):

    def __init__(self):
        super().__init__('camera')
        config = load_configuration("camera_raspberry.json", True)
        self.camera = CameraImpl(config)

        self.camera_set_fps = self.create_service(GlobalSetValue, 'set_fps', self.set_fps)
        self.camera_set_resolution = self.create_service(GlobalSetValue, 'set_resolution', self.set_resolution)
        self.camera_start_service = self.create_service(GlobalResult, 'start_camera', self.start_camera)
        self.camera_stop_service = self.create_service(GlobalResult, 'stop_camera', self.stop_camera)
        self.camera_infos_publisher = self.create_publisher(CameraInfo, 'camera_infos', 10)
        self.timer = self.create_timer(2, self.publish_camera_infos)

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
        print(f"STOP")
        try:
            self.camera.stop_server()
        except:
            success = False
        response.success = success
        return response
    
    def publish_camera_infos(self):
        """Publish parameters from the camera"""
        msg = CameraInfo()
        resolution = CameraResolution()
        resolution.width = self.camera.resolution.get("width")
        resolution.height = self.camera.resolution.get("height")
        resolution.name = self.camera.resolution.get("label")
        msg.resolution = resolution
        msg.fps = self.camera.fps
        msg.on = self.camera.is_streaming
        self.camera_infos_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_service = CameraService()

    print("camera ready")
    rclpy.spin(camera_service)    

    rclpy.shutdown()


if __name__ == '__main__':
    main()