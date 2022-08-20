import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import time
from PIL import Image
from io import BytesIO

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	"""Handle requests in a separate thread."""

class HTTPVideoStreamHandler(BaseHTTPRequestHandler):
    def __init__(self, *args):
        self.cameras = {}
        BaseHTTPRequestHandler.__init__(self, *args)

    def do_GET(self):
        if self.path.endswith('camera.mjpg'):
            print(self.path)
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            camera = self.server.camera
            for frame in camera.get_stream():
                print("serving frame{}".format(frame.shape))
                try:
                    imgRGB = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                    jpg = Image.fromarray(imgRGB)
                    tmp = BytesIO()
                    jpg.save(tmp,'JPEG')
                    self.wfile.write("--jpgboundary".encode())
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length', str(tmp.getbuffer().nbytes))
                    self.end_headers()
                    jpg.save(self.wfile,'JPEG')
                except:
                    print("done serving")
                    return

    def connection_dropped(self, error, environ=None):
        print("connection dropped")

    # def set_camera(self, camera):
    #     self.camera = camera