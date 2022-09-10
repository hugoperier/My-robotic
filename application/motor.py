from modules.motors.OsoyooMotor import OsoyooMotor
import time 


motor1 = {
    "name": "motor1",
    "speed": 2000,
    "pinForward": 27,
    "pinBackward": 22,
    "motorSpeedPort": 1
}

motor2 = {
    "name": "motor2",
    "speed": 2000,
    "pinForward": 23,
    "pinBackward": 24,
    "motorSpeedPort": 0
}

def test_motor(config):
    print("Testing motor: {}".format(config["name"]))
    motor = OsoyooMotor(config)
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
    print()

if __name__ == "__main__":
    test_motor(motor1)
    test_motor(motor2)