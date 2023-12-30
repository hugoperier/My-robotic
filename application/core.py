import argparse
from modules.utils.func_utils import load_configuration
from modules.utils.threaded_http_server import ThreadedHTTPServer
from modules.core.CoreHttpHandler import CoreHttpHandler
from modules.core.CoreModule import Core
import requests
import time

def run_server(configuration):
    port = configuration.get("port")
    server = ThreadedHTTPServer(('0.0.0.0', port), CoreHttpHandler)
    server.configuration = configuration
    server.core = Core(configuration=configuration)
    server.core.enable_status_thread()
    server.serve_forever()

    self.stop_status_thread()


def custom_link_function():
    core = Core(configuration=configuration)
    key = input("Enter the key: ")
    if (len(key) != 36):
        print("Invalid key")
        return
    
    url = f"{configuration.get('neutronCoreUri')}/agent/link"
    response = requests.post(url, {"secretKey": key})

    if response.status_code == 200:
        core.configuration["secretKey"] = key
        core.configuration["linked"] = True
        core.update_configuration()
        print("Linking successful")
    else:
        print(f"Error linking: {response.text}")

def custom_status_function(configuration):
    core = Core(configuration=configuration)

    serverUri = 'http://localhost:' + str(configuration.get('port')) + '/robot/status'
    status = "Online"
    try:
        print(serverUri)
        response = requests.get(serverUri)
        if (response.status_code != 200):
            status = "Unknown"
            return
        modules = response.json().get("modules")
        if (modules != []):
            status = "Operating"
    except:
        status = "Offline"

    systemInformations = {
        "status": status,
        "battery": core.get_battery(),
        "system": {
            "cpu": core.cpu_usage,
            "memory": core.memory_usage,
        },
        "location": {
            "name": core.get_location()
        },
        "hash": core.calculate_hash(),
        "network": {
            "hostname": core.get_network_infos()
        }
    }
    print(systemInformations)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='The neutron agent is responsible for installing and managing the services for the robot.')    

    parser.add_argument('--run', action='store_true', help='Run the server')
    parser.add_argument('--link', action='store_true', help='Link the robot with the Neutron platform')
    parser.add_argument('--status', action='store_true', help='Display robot status')

    args = parser.parse_args()
    configuration = load_configuration("core.json")

    if args.run:
        print("Starting server forever")
        run_server(configuration)
    elif args.link:
        custom_link_function()
    elif args.status:
        custom_status_function(configuration)
    else:
        parser.print_help()
    exit(0)
