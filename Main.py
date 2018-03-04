#Relies on GPIO, Adafruit DC STepper motor hat

#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import RPi.GPIO as GPIO
import time
import atexit

LeftObstaclePin = 40
RightObstaclePin = 38
BackObstaclePin = 35

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

LeftMotor = mh.getMotor(1)
RightMotor = mh.getMotor(2)
BrushMotor = mh.getMotor(4)

def setup():
	GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
	GPIO.setup(LeftObstaclePin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(RightObstaclePin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(BackObstaclePin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def loop():
	while True:
		if (0 == GPIO.input(ObstaclePin)):
			print("Detected Barrier!")
			

def destroy():
	GPIO.cleanup()                     # Release resource

if __name__ == '__main__':     # Program start from here
	setup()
	try:
		loop()
	except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
		destroy()