from __future__ import division
import time
import Adafruit_PCA9685
import RPi.GPIO as GPIO


class OsoyooMotor:
    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.speed = 2000 # Max pulse length out of 4096

        #define L298N(Model-Pi motor drive board) GPIO pins
        # self.IN1 = 23  #Left motor direction pin
        # self.IN2 = 24  #Left motor direction pin
        # self.IN3 = 27  #Right motor direction pin
        # self.IN4 = 22  #Right motor direction pin
        # self.ENA = 0  #Left motor speed PCA9685 port 0
        # self.ENB = 1  #Right motor speed PCA9685 port 1

        self.pinForward = 27
        self.pinBackward = 22
        self.motorSpeedPort = 0
        GPIO.setup(self.pinForward, GPIO.OUT)
        GPIO.setup(self.pinBackward, GPIO.OUT) 

    def changespeed(self, speed):
	    self.pwm.set_pwm(self.motorSpeedPort, 0, speed)

    def stop(self):
	    GPIO.output(self.pinBackward, GPIO.LOW)
	    GPIO.output(self.pinForward, GPIO.LOW)
	    self.changespeed(0)


    def backward(self):
	    GPIO.output(self.pinBackward, GPIO.HIGH)
	    GPIO.output(self.pinForward, GPIO.LOW)
	    self.changespeed(self.speed)
 
    def forward(self):
        GPIO.output(self.pinForward, GPIO.HIGH)
        GPIO.output(self.pinBackward, GPIO.LOW)
        self.changespeed(self.speed)