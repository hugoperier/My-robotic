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
    print("Waiting for User input: ")
    key = input()
    if key == "q":
        print("Quitting")
        break
    elif key == "w":
        print("Updating camera resolution")
        if (camera.width == 480):
            camera.width = 640
            camera.height = 480
        else:
            camera.width = 480
            camera.height = 360
    elif key == "e":
        print("Updating camera fps")
        if (camera.fps == 10):
            camera.fps = 20
        else:
            camera.fps = 10
    elif key == "r":
        print("Stopping server")
        camera.stop_server()
    elif key == "t":
        print("Starting server")
        camera.make_server(8000)