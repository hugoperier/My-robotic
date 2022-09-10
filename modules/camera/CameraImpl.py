import cv2
from .FpsCounter import FpsCounter
from threading import Thread, Lock
from .HTTPVideoStreamHandler import HTTPVideoStreamHandler, ThreadedHTTPServer

class CameraImpl:
    def __init__(self, config):
        self.__mutex__ = Lock()
        self.__server_on__ = False
        self.config = config
        self.capture = cv2.VideoCapture(config['device'], cv2.CAP_V4L)
        self.fpsCounter = FpsCounter(config['fps'])
        self.available_resolutions = config['available_resolutions']
        self.resolution = self.available_resolutions[config['default_resolution']]
        self.width = self.resolution['width']
        self.height = self.resolution['height']
        self.capture.set(cv2.CAP_PROP_SATURATION, config['saturation'])
        self.is_streaming = False
        
    @property
    def width(self):
        return self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)

    @width.setter
    def width(self, value):
        if value < 1:
            raise ValueError('Width must be greater than 0')
        with self.__mutex__:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, value)

    @property
    def height(self):
        return self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)

    @height.setter
    def height(self, value):
        if value < 1:
            raise ValueError('Height must be greater than 0')
        with self.__mutex__:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, value)

    def set_resolution(offset):
        if (offset < 0 or offset > len(self.available_resolutions)):
            raise ValueError('Camera resolution offset must be included in the available resolution list')
        self.resolution = self.available_resolutions[offset]
        self.width = self.resolution["width"]
        self.height = self.resolution["height"]

    @property
    def fps(self):
        return self.fpsCounter.fps

    @fps.setter
    def fps(self, value):
        self.fpsCounter.defined_fps = value

    def get_stream(self):
        print('Stream started')
        failCounter = 0
        self.fpsCounter.reset()
        self.is_streaming = True
        while self.is_streaming:
            self.fpsCounter.wait()
            with self.__mutex__:
                ret, frame = self.capture.read()
            if ret:
                self.fpsCounter.update()
                failCounter = 0
                yield frame
            else:
                print("could not get frame")
                failCounter += 1
                if failCounter > 10:
                    self.__reset__()
                    failCounter = 0
                continue
        print('Stream stopped')

    def __reset__(self):
        self.capture.release()
        self.capture = cv2.VideoCapture(self.config['device'], cv2.CAP_V4L)
        self.width = self.config['width']
        self.height = self.config['height']
        self.capture.set(cv2.CAP_PROP_SATURATION, self.config['saturation'])
        self.fpsCounter.reset()
        print('Camera reset')

    def make_server(self, port=8100):
        if self.__server_on__:
            raise RuntimeError('Server is already running')
        self.port = port
        self.__server_thread__ = Thread(target=self.__server_serve__)        
        self.__server_thread__.start()

    def stop_server(self):
        if self.__server_on__:
            self.__server_on__ = False
            self.stop_stream()
            print("self.__server__.socket.close()")
            self.__server__.socket.close()
            print("self.__server__.server_close()")
            self.__server__.server_close()
            print("self.__server__.shutdown()")
            self.__server__.shutdown()
            print("self.__server_thread__.join()")
            self.__server_thread__.join()
            print('Server stopped')

    def __server_serve__(self):
        print(f'Server started on port {self.port}')
        self.__server_on__ = True
        self.__server__ = ThreadedHTTPServer(('', self.port), HTTPVideoStreamHandler)
        self.__server__.camera = self
        self.__server__.serve_forever()
        print('Server stopped')
        self.__server_on__ = False

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