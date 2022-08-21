from CameraImpl import CameraImpl
from HTTPVideoStreamHandler import HTTPVideoStreamHandler, ThreadedHTTPServer
from time import sleep
import cv2

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

while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("w"):
        if (camera.width == 480):
            camera.width = 640
            camera.height = 480
        else:
            camera.width = 480
            camera.height = 360
    elif key == ord("e"):
        if (camera.fps == 10):
            camera.fps = 20
        else:
            camera.fps = 10
    elif key == ord("r"):
        camera.stop_server()
    elif key == ord("t"):
        camera.make_server(8000)