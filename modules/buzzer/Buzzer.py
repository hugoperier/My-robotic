from gpiozero import TonalBuzzer
from gpiozero.tones import Tone
from time import sleep

class Buzzer():
    def __init__(self, pin):
        self.tonal = TonalBuzzer(pin)

    @property
    def is_active(self):
        return self.tonal.is_active

    def cleanUp(self):
        self.off()

    def on(self):
        self.tonal.play(220.0)

    def off(self):        
        self.tonal.stop()

    def toggle(self):
        if (self.is_active):
            self.off()
        else:
            self.on()

    def beep(self):
        self.tonal.play(Tone("C4"))
        sleep(0.5)
        self.tonal.stop()

    def play(self, note, duration):
        self.tonal.play(Tone(note))
        sleep(duration)
        self.tonal.stop()