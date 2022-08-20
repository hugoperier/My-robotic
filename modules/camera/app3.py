from CameraImpl import CameraImpl
from HTTPVideoStreamHandler import HTTPVideoStreamHandler, ThreadedHTTPServer
from time import sleep

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
camera.make_server(8000)

sleep(5)
print(camera.fps)

sleep(2)
camera.fps = 2

sleep(5)
print(camera.fps)

sleep(2)
print("set width")
camera.width = 480
print("set height")
camera.height = 360
print("set fps")
camera.fps = 20

sleep(10)

# camera.stop_server()