from modules.utils.ProcessManager import ProcessManager
from modules.utils.func_utils import load_configuration, get_time
import psutil
import os
import json
import requests
import threading
import time
import hashlib

class Core:
    def __init__(self, configuration):
        self.stop_event = threading.Event()
        self.status_thread = None
        self.configuration = configuration
        self.name = self.configuration.get("name")
        self.type = self.configuration.get("type")
        self.connection = self.configuration.get("connection")
        self.context = load_configuration(f"contexts/{configuration.get('context').get('type')}.json")
        self.context["process"]["processId"] = None
        self.parts = [x for x in self.configuration.get("parts", [])]
        self.process_manager = None

    def start_robot(self):
        #1. Configure for session
        self.process_manager = ProcessManager()
        self.process_manager.start()
        self.operation_time = -1

        #2. start the context
        
        processId = self.process_manager.make_process(
            self.context["process"].get("name"),
            self.context["process"].get("command"),
            self.context["process"].get("path"),
            self.context["process"].get("id"),
            self.context["process"].get("initializer"),
            True
        )
        context = self.process_manager.get_process(processId)
        context.waitForReady()
        self.context["process"]["processId"] = processId
        print("Context has successfuly been created")

        #3. start robot processes

        for part in self.parts:
            command = f"ros2 run {part.get('ros2Package')} {part.get('ros2Node')}"
            processId = self.process_manager.make_process(
                part.get("name"),
                command,
                "/home/hugoperier/projects/My-robotic",
                part.get("id"),
                part.get("initializer", None),
                True
            )
            part["processId"] = processId

    def stop_robot(self):
        # Stop robot processes
        for part in self.parts:
            processId = part["processId"]
            self.process_manager.stop_process(part["processId"], True)
            part["processId"] = None

        # Stop context
        processId = self.context["process"]["processId"]
        if (processId != None):
            self.process_manager.stop_process(processId, True)
            processId = None

        self.process_manager.stop()


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

    def get_battery(self):
        battery = {
            "level": 80,
            "charging": False
        }
        return battery

    def enable_status_thread(self):
        # Create and start a thread that runs send_robot_status every 5 minutes
        self.status_thread = threading.Thread(target=self.status_thread_function)
        self.status_thread.start()

    def status_thread_function(self):
        while not self.stop_event.is_set():
            # Perform the status update
            self.send_robot_status()

            # Sleep for 5 minutes
            time.sleep(300)  # 300 seconds = 5 minutes

    def stop_status_thread(self):
        if (self.status_thread == None):
            return

        # Set the stop event to signal the thread to stfop
        self.stop_event.set()

        # Wait for the thread to finish
        self.status_thread.join()

    def get_network_infos(self):
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        hostname = s.getsockname()[0]
        s.close()
        return hostname

    def calculate_hash(self):
        obj = {
            "name": self.configuration["name"],
            "context": self.configuration["context"],
            "parts": self.configuration["parts"]
        }
        json_str = json.dumps(obj, separators=(',', ':'))
        return hashlib.sha256(json_str.encode()).hexdigest()

    def send_robot_status(self):
        if (self.configuration.get("linked") != True):
            print("Error: The robot must be linked to send status to the server")
            return

        serverUri = str(self.configuration.get('neutronCoreUri')) + '/agent/publishSystemInformation'
        systemInformations = {
                "status": self.status,
                "battery": self.get_battery(),
                "system": {
                    "cpu": self.cpu_usage,
                    "memory": self.memory_usage,
                },
                "location": {
                    "name": self.get_location()
                },
                "hash": self.calculate_hash(),
                "network": {
                    "hostname": self.get_network_infos()
                }
            }
        requestPayload = {
            "secretKey": self.configuration.get("secretKey"),
            "status": systemInformations
        }

        try:
            response = requests.post(serverUri, json=requestPayload)
            body = response.json()
            print("Response include ")
            print(body)
            print("Published status to neutron server")
            if (body.get("configuration")):
                self.pull_configuration(body["configuration"])
            if (response.status_code != 200):
                print("Error while publishing robot status")
                print(response.text)
        except Exception as e:
            print("[CORE] Error sending robot status")
            print(str(e))
        

    def pull_configuration(self, configuration):
        print("Pulling new configuration")
        print(configuration)
        print()
        self.configuration["context"] = configuration["name"]
        self.configuration["context"] = configuration["context"]
        self.configuration["parts"] = configuration["parts"]
        self.update_configuration()
        self.context = load_configuration(f"contexts/{configuration.get('context').get('type')}.json")

    def update_configuration(self):
        try:
            myrobotics_root = os.environ.get("MYROBOTICS_ROOT")
            if not myrobotics_root:
                raise ValueError("MYROBOTICS_ROOT environment variable not set")

            config_path = os.path.join(myrobotics_root, "core.json")

            with open(config_path, "w") as config_file:
                json.dump(self.configuration, config_file, indent=4)

            print(f"[CORE] Configuration updated and saved to {config_path}")

        except Exception as e:
            print("[CORE] Error updating configuration")
            print(str(e))


    def get_location(self):
        return "Unknown"

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
        if (self.process_manager and len(self.process_manager.processes)):
            return "Operating"
        return "Online"
