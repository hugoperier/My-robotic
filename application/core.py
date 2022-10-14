from modules.utils.func_utils import load_configuration
from modules.utils.threaded_http_server import ThreadedHTTPServer
from modules.core.CoreHttpHandler import CoreHttpHandler
from modules.utils.ProcessManager import ProcessManager

if __name__ == '__main__':
	print("starting forever")
	configuration = load_configuration("/home/pi/.myrobotics/core.json")

	port = configuration.get("port")
	server = ThreadedHTTPServer(('0.0.0.0', port), CoreHttpHandler)
	server.configuration = configuration
	server.process_manager = ProcessManager()
	server.process_manager.start()
	server.serve_forever()