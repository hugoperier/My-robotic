from modules.motors.OsoyooMotor import OsoyooMotor

class OsoyooBase:
    def __init__(self, configuration):
        self.i = 0
        self.configuration = configuration

        self.wheels = []
        for wheel in configuration["wheels"]:
            self.wheels.append(OsoyooMotor(wheel))
            self.speed = wheel["speed"]
        
    def set_speed(self, speed):
        for wheel in self.wheels:
            wheel.changespeed(speed)
        self.speed = speed
        
    def forward(self):
        for wheel in self.wheels:
            wheel.forward()

    def backward(self):
        for wheel in self.wheels:
            wheel.backward()

    def stop(self):
        for wheel in self.wheels:
            wheel.stop()

    def left(self):
        self.wheels[0].backward()
        self.wheels[1].forward()

    def right(self):
        self.wheels[0].forward()
        self.wheels[1].backward()