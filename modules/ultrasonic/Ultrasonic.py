from gpiozero import DistanceSensor
from time import sleep

class Ultrasonic():
    def __init__(self):
        self.sensor = DistanceSensor(echo=21, trigger=16)

    @property
    def distance(self):
        return self.sensor.distance * 100