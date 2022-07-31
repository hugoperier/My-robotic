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
        self.IN1 = 23  #Left motor direction pin
        self.IN2 = 24  #Left motor direction pin
        self.IN3 = 27  #Right motor direction pin
        self.IN4 = 22  #Right motor direction pin
        self.ENA = 0  #Left motor speed PCA9685 port 0
        self.ENB = 1  #Right motor speed PCA9685 port 1

        # Define motor control  pins as output
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT) 
        GPIO.setup(self.IN3, GPIO.OUT) 
        GPIO.setup(self.IN4, GPIO.OUT)

    def changespeed(self, speed):
        self.pwm.set_pwm(self.ENA, 0, speed)
        self.pwm.set_pwm(self.ENB, 0, speed)

    def stopcar(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.changespeed(0)

    def backward(self):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.changespeed(self.speed)

    def forward(self):
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        self.changespeed(self.speed)

    def turnRight(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.changespeed(self.speed)
    
    def turnLeft(self):
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        self.changespeed(self.speed)