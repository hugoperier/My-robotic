from modules.utils.ProcessManager import ProcessManager
from modules.utils.func_utils import load_configuration, get_time
import psutil
import os

class Core:
    def __init__(self, configuration):
        self.configuration = configuration
        self.name = self.configuration.get("name")
        self.type = self.configuration.get("type")
        self.connection = self.configuration.get("connection")
        self.modules = [x for x in self.configuration.get("modules", [])]
        try:
            self.modules = list(map(lambda x: {**x, 'configuration': load_configuration(x.get(
                'configuration'))} if type(x.get('configuration')) == str else x, self.modules))
        except Exception as e:
            print("[CORE] Error while initializing, existing program")
            print(str(e))
            exit(1)
        self.process_manager = ProcessManager()
        self.process_manager.start()
        self.operation_time = -1

    def make_process(self, processId):
        processInfos = [x for x in self.configuration.get(
            "modules", []) if x["id"] == processId]
        if (len(processInfos) == 0):
            raise ValueError("No valid process id")
        processInfo = processInfos[0]
        process_id = self.process_manager.make_process(processInfo.get('name'), processInfo.get(
            "command"), processInfo.get("path"), processId, initializer=processInfo.get("initializer"), pipe=True)
        process = self.process_manager.get_process(process_id)
        process.waitForReady()
        if (self.operation_time == -1 and len(self.process_manager.processes)):
            self.operation_time = get_time()
        return process_id

    def stop_process(self, processId, flush):
        success = self.process_manager.stop_process(processId, flush)
        if not (len(self.process_manager.processes)):
            self.operation_time = -1
        if not (success):
            raise ValueError("No valid process id")

    def get_enabled_modules(self):
        modules = [x for x in self.modules if x.get("enabled") == True]
        modules = map(lambda x: {"id": x.get("id"), "name": x.get("name"), "type": x.get(
            "type"), "framePackage": x.get("framePackage"), "configuration": x.get("configuration")}, modules)
        return modules

    def get_processes_status(self):
        processes = []
        for i in self.process_manager.processes:
            processes.append(i.get_info())
        return processes

    @property
    def cpu_usage(self):
        return psutil.cpu_percent()

    @property
    def memory_usage(self):
        return psutil.virtual_memory().percent

    @property
    def status(self):
        if (len(self.process_manager.processes)):
            return "Connected"
        return "Available"
