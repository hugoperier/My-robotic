from OsoyooMotor import OsoyooMotor
import time 

motors = OsoyooMotor()
motors.forward()
time.sleep(2)  

motors.stopcar()
time.sleep(2.25)

motors.backward()
time.sleep(2)  
motors.stopcar()
time.sleep(2.25) 

motors.turnLeft()
time.sleep(2)  
motors.stopcar()
time.sleep(2.25)
	
motors.turnRight()
time.sleep(5)  
motors.stopcar()
time.sleep(2.25)