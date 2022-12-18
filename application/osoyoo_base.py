from modules.motors.OsoyooBase import OsoyooBase
import time
from modules.utils.func_utils import load_configuration

def test_base():
    configuration = load_configuration("./osoyoo_base.json")
    base = OsoyooBase(configuration)

    v1 = 0
    v2 = 0
    while True:
        print(f"x: {v1} yaw: {v2}")
        print("Waiting for User input: ")
        key = input()
        print("input: " + key)
        if key == "q":
            print("Quitting")
            break
        elif key == "s":
            v1 = int(input('Enter x: '))
            v2 = int(input('Enter yaw: '))
        elif key == "r":
            print("r")
            base.set_velocity(v1, v2)
        elif key == "q":
            base.stop()
        elif key == "l":
            break

if __name__ == "__main__":
    test_base()