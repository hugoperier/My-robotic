from OsoyooMotor import OsoyooMotor
import time 

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