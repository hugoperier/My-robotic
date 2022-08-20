import cv2
import threading
from threading import Thread
from socketserver import ThreadingMixIn
from CameraHandler import CamHandler, ThreadedHTTPServer
import time

class Camera:
    def __init__(self):
        self.capture = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.capture.set(cv2.CAP_PROP_SATURATION, 0.2)
        self.streamer = {
            'server' : None,
            'port' : 5000,
            'enabled' : False
        }

    def getStream(self):
        if (self.streamer["enabled"] == False):
            handler = lambda *args: CamHandler(*args, capture=self.capture)
            self.streamer["server"] = ThreadedHTTPServer(('0.0.0.0', self.streamer["port"]), handler)
            self.streamer["enabled"] = True
            self.streamer["thread"] = Thread(target=self.createStream, args=(self.streamer["server"], self.streamer["enabled"]))
            self.streamer["thread"].start()
        else:
            print(f"An existing streamer is active on ({self.streamer['port']})")

    def stopStream(self):
        if (self.streamer['server'] is not None):
            print("cs")
            self.streamer['server'].socket.close()
            print("cs3")
            self.streamer['server'].server_close()
            print("cs4")
            self.streamer['server'].shutdown()
        print("cs2")
        if (self.capture is not None):
            self.capture.release()
        self.streamer['enabled'] = False
        print(self.streamer["enabled"])
        if (self.streamer["thread"] is not None):
            self.streamer["thread"].join()
        print("server is closed")

    def createStream(self, server, enabled):
        print("Server started at port ", self.streamer["port"])        
        while (enabled == True):
            print(enabled)
            server.handle_request()
        print("I quit")

    def screenshot(path):
        print("Not yet")