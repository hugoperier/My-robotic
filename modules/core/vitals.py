from time import sleep
import psutil

def get_vitals():
    return {
        "cpu": psutil.cpu_percent(),
        "ram": psutil.virtual_memory().percent
    }

    while True:
        vitals = get_vitals()
        sleep(0.5)
        print("tick", vitals)

