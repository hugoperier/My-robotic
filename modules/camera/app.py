from Camera import Camera
from time import sleep

cam = Camera()

print("get stream")
cam.getStream()

sleep(10)
print("stop stream")
cam.stopStream()