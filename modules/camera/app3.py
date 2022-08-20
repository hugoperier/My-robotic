# from CameraImpl import CameraImpl
from FpsCounter import FpsCounter
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

print("get stream 1")
while True:
    for frame in camera.getStream():
        print("Frame received, size: {}, fps {}".format(frame.shape, camera.fpsCounter.fps))

print("bye")