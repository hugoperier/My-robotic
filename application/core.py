from modules.utils.func_utils import load_configuration
from modules.utils.threaded_http_server import ThreadedHTTPServer
from modules.core.core import HTTPCoreServerStreamHandler
from modules.utils.ProcessManager import ProcessManager

if __name__ == '__main__':
	print("starting forever")
	configuration = load_configuration("./configuration/core.json")

	port = configuration.get("port")
	server = ThreadedHTTPServer(('0.0.0.0', port), HTTPCoreServerStreamHandler)
	server.configuration = configuration
	server.process_manager = ProcessManager()
	server.process_manager.start()
	server.serve_forever()