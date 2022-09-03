from Ultrasonic import Ultrasonic
from modules.ultrasonic.Ultrasonic import Ultrasonic
from time import sleep

if __name__ == 'main':
    sensor = Ultrasonic()
    while True:
        print('Distance: ', sensor.distance)
        sleep(1)