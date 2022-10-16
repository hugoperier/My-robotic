from modules.utils.func_utils import load_configuration
from modules.utils.threaded_http_server import ThreadedHTTPServer
from modules.core.CoreHttpHandler import CoreHttpHandler
from modules.utils.ProcessManager import ProcessManager
from modules.core.CoreModule import Core

if __name__ == '__main__':
	print("starting forever")
	configuration = load_configuration("core.json")

	port = configuration.get("port")
	server = ThreadedHTTPServer(('0.0.0.0', port), CoreHttpHandler)
	server.configuration = configuration
	server.core = Core(configuration=configuration)
	server.serve_forever()