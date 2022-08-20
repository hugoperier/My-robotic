#!/usr/bin/python
'''
	Author: Igor Maculan - n3wtron@gmail.com
	A Simple mjpg stream http server
'''
import cv2
from PIL import Image
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from io import StringIO, BytesIO
import time
capture = None


class CamHandler(BaseHTTPRequestHandler):
	def do_GET(self):
		if self.path.endswith('.mjpg'):
			self.send_response(200)
			self.send_header(
			    'Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
			self.end_headers()
			prev_frame_time = 0
 
			# used to record the time at which we processed current frame
			new_frame_time = 0
			while True:
				try:
					rc, img = capture.read()
					if not rc:
						continue
					imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
					jpg = Image.fromarray(imgRGB)
					tmpFile = BytesIO()
					jpg.save(tmpFile, 'JPEG')
					new_frame_time = time.time()
					fps = 1/(new_frame_time-prev_frame_time)
					prev_frame_time = new_frame_time
 
    				# converting the fps into integer
					fps = int(fps)
 
    				# converting the fps to string so that we can display it on frame
    				# by using putText function
					fps = str(fps)
					print("send", fps)
					self.wfile.write("--jpgboundary".encode())
					self.send_header('Content-type','image/jpeg')
					self.send_header('Content-length',str(tmpFile.getbuffer().nbytes))
					self.end_headers()
					jpg.save(self.wfile,'JPEG')
				except KeyboardInterrupt:
					break
			return
		if self.path.endswith('.html'):
			self.send_response(200)
			self.send_header('Content-type','text/html')
			self.end_headers()
			self.wfile.write('<html><head></head><body>'.encode())
			self.wfile.write('<img src="http://127.0.0.1:8087/cam.mjpg"/>'.encode())
			self.wfile.write('</body></html>'.encode())
			return


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	"""Handle requests in a separate thread."""

def main():
	global capture
	capture = cv2.VideoCapture('/dev/video0')
	capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320); 
	capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240);
	capture.set(cv2.CAP_PROP_SATURATION,0.2);
	global img
	try:
		server = ThreadedHTTPServer(('0.0.0.0', 8087), CamHandler)
		print( "server started")
		server.serve_forever()
	except KeyboardInterrupt:
		capture.release()
		server.socket.close()

if __name__ == '__main__':
	main()