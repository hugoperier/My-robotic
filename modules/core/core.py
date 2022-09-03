from http.server import BaseHTTPRequestHandler, HTTPServer
from ProcessManager.manager import ProcessManager
from socketserver import ThreadingMixIn
import json

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
 	"""Handle requests in a separate thread."""

class HTTPCoreServerStreamHandler(BaseHTTPRequestHandler):
	def __init__(self, *args):
		BaseHTTPRequestHandler.__init__(self, *args)
	
	def do_GET(self):
		print("req")
		if self.path == '/status':
			self.status()
		elif self.path == '/list':
			print("list")
			self.list_process()

	def do_POST(self):
		body = self.rfile.read(int(self.headers['Content-Length']))
		self.body = json.loads(body.decode())
		print("body", self.body)

		if self.path == '/start':
			self.start_process()
		elif self.path == '/stop':
			self.stop_process()
		elif self.path == '/connect':
			self.create_connection()

	def status(self):
		self.send_response(200)
		self.send_header('Content-type', 'application/json')
		self.end_headers()
		
	def list_process(self):
		self.send_response(200)
		processes = []
		for i in self.server.process_manager.processes:
			processes.append(i.get_info())
		self.send_header('Content-type', 'application/json')
		self.end_headers()
		self.wfile.write(json.dumps(processes).encode())

	def start_process(self):
		if (self.body == None or self.body == "" or self.body == {} or self.body == []
			or self.body.get("processId") == None or self.body.get("name") == None):
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.end_headers()
			self.wfile.write("No valid body".encode())
			return
		process_id = self.body['processId']
		processInfos = [x for x in self.server.configuration.get("modules", []) if x["id"] == process_id]
		if not (len(processInfos) > 0):
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.end_headers()
			self.wfile.write("No valid process id".encode())
			return

		try:
			process_id = self.server.process_manager.make_process(self.body['name'], processInfos[0].get("command"), processInfos[0].get("path"))
		except ValueError as e:
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.end_headers()
			self.wfile.write("Process not stopped: {}".format(e).encode())

		self.send_response(200)
		self.send_header('Content-type', 'application/json')
		self.end_headers()
		self.wfile.write(json.dumps({"processId": process_id}).encode()) 
		

	def stop_process(self):
		process_id = self.body.get("processId")
		if (self.body == None or self.body == "" or self.body == {} or self.body == []
			or process_id == None):
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.end_headers()
			self.wfile.write("No valid body".encode())
			return
		process_id = self.body['processId']
		flush = self.body.get("flush", False)
		success = self.server.process_manager.stop_process(process_id, flush)
		if not (success):
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.end_headers()
			self.wfile.write("No valid process id".encode())
			return
		self.send_response(200)
		self.end_headers()

def load_configuration():
	with open("./configuration/core.json") as json_file:
		return json.load(json_file)

if __name__ == '__main__':
	print("starting forever")
	configuration = load_configuration()

	port = configuration.get("port")
	server = ThreadedHTTPServer(('0.0.0.0', port), HTTPCoreServerStreamHandler)
	server.configuration = configuration
	server.process_manager = ProcessManager()
	server.process_manager.start()
	server.serve_forever()