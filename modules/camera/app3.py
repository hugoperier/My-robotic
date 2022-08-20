from CameraImpl import CameraImpl
from HTTPVideoStreamHandler import HTTPVideoStreamHandler, ThreadedHTTPServer
import time

config = {
    'device': "/dev/video0",
    'max_width': 1920,
    'max_height': 1080,
    'fps': 20,
    'width': 640,
    'height': 480,
    'saturation': 0.5
}

camera = CameraImpl(config)
httpd = ThreadedHTTPServer(('', 8000), HTTPVideoStreamHandler)
httpd.camera = camera
print("Server started")
httpd.handle_request()

print("bye")