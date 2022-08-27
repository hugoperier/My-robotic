from http.server import BaseHTTPRequestHandler, HTTPServer

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	"""Handle requests in a separate thread."""

class HTTPVideoStreamHandler(BaseHTTPRequestHandler):