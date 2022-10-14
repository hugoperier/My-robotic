from modules.utils.HttpRequestHandler import HttpRequestHandler

class CoreHttpHandler(HttpRequestHandler):
    def __init__(self, *args):
        get = {
            "/robot/status": self.robot_status,
            "/robot/configuration": self.robot_configuration
        }
        post = {
            "/start": self.start_process,
            "/stop": self.stop_process,
            "/connect": self.create_connection
        }
        HttpRequestHandler.__init__(self, get, post, *args)
        # self.add_route("GET", "/robot/status", self.robot_status)
        # self.add_route("GET", "/robot/configuration", self.robot_configuration)
        # self.add_route("POST", "/start", self.start_process)
        # self.add_route("POST", "/stop", self.stop_process)
        # self.add_route("POST", "/connect", self.create_connection)
        # self.add_routes(get, post)

    def robot_status(self, t):
        print("robot_status")
        self.success()

    def robot_configuration(self):
        print("robot_configuration")
        self.success()

    def start_process(self):
        print("start_process")
        self.success()
    
    def stop_process(self):
        print("stop_process")
        self.success()
    
    def create_connection(self):
        print("create_connection")
        self.success()
