import RPi.GPIO as GPIO
import time
import cv2

pwm1 = 32
pwm2 = 33
dir1 = 36
dir2 = 37

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pwm1, GPIO.OUT)
GPIO.setup(pwm2, GPIO.OUT)
GPIO.setup(dir1, GPIO.OUT)
GPIO.setup(dir2, GPIO.OUT)

def init(pw = 25, cc = 25):
    global power
    global cycle
    power = pw
    cycle = cc

    global p1
    global p2
    p1 = GPIO.PWM(pwm1, power)
    p2 = GPIO.PWM(pwm2, power)

def pins(pw1, pw2, dr1, dr2):
    pwm1 = pw1
    pwm2 = pw2
    dir1 = dr1
    dir2 = dr2

def stop():
    p1.stop()
    p2.stop()

def forward():
    GPIO.output(dir1, GPIO.LOW)
    GPIO.output(dir2, GPIO.LOW)

    p1.start(cycle)
    p2.start(cycle)

def backward():
    GPIO.output(dir1, GPIO.HIGH)
    GPIO.output(dir2, GPIO.HIGH)

    p1.start(cycle)
    p2.start(cycle)

def left():
    GPIO.output(dir1, GPIO.LOW)
    GPIO.output(dir2, GPIO.HIGH)

    p1.start(cycle)
    p2.start(1.0)

def right():
    GPIO.output(dir1, GPIO.HIGH)
    GPIO.output(dir2, GPIO.LOW)

    p1.start(1.0)
    p2.start(cycle)
