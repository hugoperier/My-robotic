from http.server import BaseHTTPRequestHandler, HTTPServer
from modules.utils.ProcessManager import ProcessManager
from socketserver import ThreadingMixIn
import json

class HTTPCoreServerStreamHandler(BaseHTTPRequestHandler):
	def __init__(self, *args):
		BaseHTTPRequestHandler.__init__(self, *args)
	
	def do_GET(self):
		if self.path == '/robot/status':
			self.robot_status()
		elif self.path == '/robot/configuration':
			self.robot_configuration()
		else:
			self.send_response(404)
			self.send_header('Content-type', 'text/html')
			self.send_header('Access-Control-Allow-Origin', '*')
			self.end_headers()

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
		else:
			self.send_response(404)
			self.send_header('Content-type', 'text/html')
			self.send_header('Access-Control-Allow-Origin', '*')
			self.end_headers()

	def robot_configuration(self):
		modules = [x for x in self.server.configuration.get("modules", []) if x.get("enabled") == True]
		modules = map(lambda x: {"id": x.get("id"), "name": x.get("name"), "type": x.get("type")}, modules)
		status = "Available"
		if (len(self.server.process_manager.processes)):
			status = "Connected"
		configuration = {
			"robot": {
				"battery": 100,
				"name": self.server.configuration.get("name"),
				"type": self.server.configuration.get("type"),
				"status": status,
				"connection": self.server.configuration.get("connection"),
				"modules": list(modules)
			}
		}
		self.send_response(200)
		self.send_header('Content-type', 'application/json')
		self.send_header('Access-Control-Allow-Origin', '*')
		self.end_headers()
		self.wfile.write(json.dumps(configuration).encode())

	def robot_status(self):
		processes = []
		for i in self.server.process_manager.processes:
			processes.append(i.get_info())
		self.send_response(200)
		self.send_header('Content-type', 'application/json')
		self.send_header('Access-Control-Allow-Origin', '*')
		self.end_headers()
		self.wfile.write(json.dumps(processes).encode())

	def start_process(self):
		if (self.body == None or self.body == "" or self.body == {} or self.body == []
			or self.body.get("processId") == None or self.body.get("name") == None):
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.send_header('Access-Control-Allow-Origin', '*')
			self.end_headers()
			self.wfile.write("No valid body".encode())
			return
		process_id = self.body['processId']
		processInfos = [x for x in self.server.configuration.get("modules", []) if x["id"] == process_id]
		if not (len(processInfos) > 0):
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.send_header('Access-Control-Allow-Origin', '*')
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
		self.send_header('Access-Control-Allow-Origin', '*')
		self.end_headers()
		self.wfile.write(json.dumps({"processId": process_id}).encode()) 
		

	def stop_process(self):
		process_id = self.body.get("processId")
		if (self.body == None or self.body == "" or self.body == {} or self.body == []
			or process_id == None):
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.send_header('Access-Control-Allow-Origin', '*')
			self.end_headers()
			self.wfile.write("No valid body".encode())
			return
		process_id = self.body['processId']
		flush = self.body.get("flush", False)
		success = self.server.process_manager.stop_process(process_id, flush)
		if not (success):
			self.send_response(400)
			self.send_header('Content-type', 'text/html')
			self.send_header('Access-Control-Allow-Origin', '*')
			self.end_headers()
			self.wfile.write("No valid process id".encode())
			return
		self.send_response(200)
		self.end_headers()