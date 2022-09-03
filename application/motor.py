from modules.motors.OsoyooMotor import OsoyooMotor
import time 

def test_motor():
    motors = OsoyooMotor()
    print("move forward")
    motors.forward()
    time.sleep(2)  
    print("stop")
    motors.stop()
    time.sleep(2.25)
    print("move backward")
    motors.backward()
    time.sleep(2)
    print("stop")  
    motors.stop()
    time.sleep(2.25) 

if __name__ == "__main__":
    test_motor()