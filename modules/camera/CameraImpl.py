import cv2
from FpsCounter import FpsCounter

class CameraImpl:
    def __init__(self, config):
        self.capture = cv2.VideoCapture(config['device'], cv2.CAP_V4L)
        self.max_width = config['max_width']
        self.max_height = config['max_height']
        self.fpsCounter = FpsCounter(config['fps'])
        self.width = config['width']
        self.height = config['height']
        self.capture.set(cv2.CAP_PROP_SATURATION, config['saturation'])
        self.is_streaming = False
        
    @property
    def width(self):
        return self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)

    @width.setter
    def width(self, value):
        if value < 1:
            raise ValueError('Width must be greater than 0')
        if value > self.max_width:
            raise ValueError('Width must be less than maximum {}'.format(self.max_width))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, value)

    @property
    def height(self):
        return self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    @height.setter
    def height(self, value):
        if value < 1:
            raise ValueError('Height must be greater than 0')
        if value > self.max_height:
            raise ValueError('Height must be less than maximum {}'.format(self.max_height))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, value)

    @property
    def fps(self):
        return self.fpsCounter.fps

    @fps.setter
    def fps(self, value):
        self.fpsCounter.defined_fps = value

    def get_stream(self):
        print('Stream started')
        self.fpsCounter.reset()
        self.is_streaming = True
        while self.is_streaming:
            self.fpsCounter.wait()
            ret, frame = self.capture.read()
            if ret:
                self.fpsCounter.update()
                yield frame
            else:
                print("error")
                break
        print('Stream stopped')

    def stop_stream(self):
        print('Stream stopping')
        self.is_streaming = False

    def screenshot(self, path):
        if self.is_streaming:
            frame = next(self.getStream())
        else:
            ret, frame = self.capture.read()
            if not ret:
                raise RuntimeError('Could not read frame')
        cv2.imwrite(path, frame)