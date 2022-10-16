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
        }
        HttpRequestHandler.__init__(self, get, post, *args)

    def robot_status(self):
        status = self.server.core.get_processes_status()
        self.send_json(status)

    def robot_configuration(self):
        configuration = self.server.core.get_configuration()
        self.send_json(configuration)

    def start_process(self):
        try:
            print("start_process")
            process_id = self.server.core.make_process(self.body.get("processId"))
            self.send_json(data={"processId": process_id})
        except ValueError as e:
            print(str(e))
            self.send_error(400, str(e))
    
    def stop_process(self):
        try:
            process_id = self.body.get("processId")
            flush = self.body.get("flush")
            self.server.core.stop_process(process_id, flush)
            self.success()
        except ValueError as e:
            self.send_error(400, str(e))
    