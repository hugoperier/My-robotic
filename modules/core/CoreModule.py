from modules.utils.ProcessManager import ProcessManager
from modules.utils.func_utils import load_configuration

class Core:
    def __init__(self, configuration):
        self.process_manager = ProcessManager()
        self.process_manager.start()
        self.configuration = configuration

    def make_process(self, processId):
        print(self.configuration.get("modules", []), processId)
        processInfos = [x for x in self.configuration.get("modules", []) if x["id"] == processId]
        if (len(processInfos) == 0):
            raise ValueError("No valid process id")
        processInfo = processInfos[0]
        process_id = self.process_manager.make_process(processInfo.get('name'), processInfo.get("command"), processInfo.get("path"), processId)
        return process_id

    def stop_process(self, processId, flush):
        success = self.process_manager.stop_process(processId, flush)
        if not (success):
            raise ValueError("No valid process id")

    def get_configuration(self):
        modules = [x for x in self.configuration.get("modules", []) if x.get("enabled") == True]
        modules = map(lambda x: {"id": x.get("id"), "name": x.get("name"), "type": x.get("type")}, modules)
        status = "Available"
        if (len(self.process_manager.processes)):
            status = "Connected"
        configuration = {
            "robot": {
                "battery": -1,
                "name": self.configuration.get("name"),
                "type": self.configuration.get("type"),
                "status": status,
                "connection": self.configuration.get("connection"),
                "modules": list(modules)
            }
        }
        return configuration

    def get_processes_status(self):
        processes = []
        for i in self.process_manager.processes:
            processes.append(i.get_info())
        return processes