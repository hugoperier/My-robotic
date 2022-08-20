from CameraImpl import CameraImpl

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
    for frame in camera.getStream():
        print("Frame received, size: {}, fps {}".format(frame.shape, camera.fpsCounter.fps))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    break