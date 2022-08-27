from tqdm import tqdm
from time import sleep
import psutil

def get_vitals():
    return {
        "cpu": psutil.cpu_percent(),
        "ram": psutil.virtual_memory().percent
    }

with tqdm(total=100, desc='cpu%', position=1) as cpubar, tqdm(total=100, desc='ram%', position=0) as rambar:
    while True:
        vitals = get_vitals()
        rambar.n= vitals["ram"]
        cpubar.n= vitals["cpu"]
        rambar.refresh()
        cpubar.refresh()
        sleep(0.5)
        print("tick")

