from modules.ultrasonic.Ultrasonic import Ultrasonic
from modules.buzzer.Buzzer import Buzzer
from time import sleep

ultrasonic = Ultrasonic()
buzzer = Buzzer(23)

while(1):
    if (ultrasonic.distance != 100):
        buzzer.beep()
    sleep(0.05)
    print(ultrasonic.distance)