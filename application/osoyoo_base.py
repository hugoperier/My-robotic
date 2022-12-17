from modules.motors.OsoyooBase import OsoyooBase
import time
from modules.utils.func_utils import load_configuration

def test_base():
    configuration = load_configuration("./osoyoo_base.json")
    base = OsoyooBase(configuration)

    purcent = 50
    while True:
        print("Waiting for User input: ")
        key = input()
        print("input: " + key)
        if key == "p":
            print("Quitting")
            break
        elif key == "w":
            print("front")
            base.forward()
        elif key == "q":
            print("left")
            base.left()
        elif key == "s":
            print("back")
            base.backward()
        elif key == "e":
            print("right")
            base.right()
        elif key == "f":
            print("stop")
            base.stop()
        elif key == 'o':
            print("speed up")
            base.set_speed(base.speed + 100)
        elif key == 'l':
            print("speed down")
            base.set_speed(base.speed - 100)
        elif key == 'i':
            print("speed up")
            purcent += 10
            base.set_speed_purcent(purcent)
        elif key == 'k':
            print("speed down")
            purcent -= 10
            base.set_speed_purcent(purcent)
        


if __name__ == "__main__":
    test_base()