from modules.camera.CameraImpl import CameraImpl
from modules.camera.HTTPVideoStreamHandler import HTTPVideoStreamHandler
from modules.utils.func_utils import load_configuration
from time import sleep
import cv2

def test_camera():
    config = load_configuration("camera_raspberry.json")

    camera = CameraImpl(config)

    while True:
        print("Waiting for User input: ")
        key = input()
        print("input: " + key)
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

if __name__ == "__main__":
    test_camera()