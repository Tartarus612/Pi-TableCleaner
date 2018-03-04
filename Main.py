#Relies on GPIO, Adafruit DC STepper motor hat, transitions https://github.com/pytransitions/transitions

#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import RPi.GPIO as GPIO
import time
import atexit
import random
from transitions import Machine

class VacuumStateMachine(object):
    #define states for vacuum
    states = ['waiting for surface', 'starting', 'cleaning table surface', 'cleaning surface turn', 'cleaning table edges', 'cleaning edges turn', 'off']
    def __init__(self):

        # Initialize the state machine
        self.machine = Machine(model=self, states=VacuumStateMachine.states, initial='off')
        
        #edges found while cleaning surface
        self.surfaceEdgesFound = 0
        #edges found while cleaning edges
        self.edgeEdgesFound = 0
        
        #number of surface edges to find to be done cleaning
        self.surfaceEdgesNeeded = 10
        #number of edge edges to find to be done cleaning
        self.edgeEdgesNeeded = 4

        # Add some transitions. We could also define these using a static list of
        # dictionaries, as we did with states above, and then pass the list to
        # the Machine initializer as the transitions= argument.

        # powered on.
        self.machine.add_transition(trigger='poweredOn', source='off', dest='waiting for surface')
        # placed on surface.
        self.machine.add_transition(trigger='surfaceDetected', source='waiting for surface', dest='starting')
        # started running found a side.
        self.machine.add_transition(trigger='edgeDetected', source='starting', dest='cleaning surface turn', after='SurfaceEdgeFound')
        # cleaning surface and found edge.
        self.machine.add_transition(trigger='edgeDetected', source='cleaning table surface', dest='cleaning table edges', conditions=['FinishedSurface'])
        self.machine.add_transition(trigger='edgeDetected', source='cleaning table surface', dest='cleaning surface turn', after='SurfaceEdgeFound')
        self.machine.add_transition(trigger='turnCompleted', source='cleaning surface turn', dest='cleaning table surface')
        # edges cleaned.
        self.machine.add_transition(trigger='edgeDetected', source='cleaning table edges', dest='off', conditions=['FinishedEdges'])
        self.machine.add_transition(trigger='edgeDetected', source='cleaning table edges', dest='cleaning edges turn', after='EdgeEdgeFound')
        self.machine.add_transition(trigger='turnCompleted', source='cleaning edges turn', dest='cleaning table edges')
        # waiting for surface again.
        self.machine.add_transition(trigger='pickedUp', source='off', dest='waiting for surface')

    def FinishedSurface(self):
        if self.surfaceEdgesFound == self.surfaceEdgesNeeded:
            return True
        return False
    
    def FinishedEdges(self):
        if self.edgeEdgesFound == self.edgeEdgesNeeded:
            return True
        return False
    
    def SurfaceEdgeFound(self):
        self.surfaceEdgesFound += 1

    def EdgeEdgeFound(self):
        self.edgeEdgesFound += 1
        

vacuumState = VacuumStateMachine()

leftObstaclePin = 40
rightObstaclePin = 38
backObstaclePin = 35
baseSpeed = 200
slowSpeed = 50

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

leftMotor = mh.getMotor(1)
rightMotor = mh.getMotor(2)
brushMotor = mh.getMotor(4)

def setup():
	GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
	GPIO.setup(leftObstaclePin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(rightObstaclePin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(backObstaclePin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def detectSurface():
    if (0 == GPIO.input(leftObstaclePin) and 0 == GPIO.input(rightObstaclePin) and 0 == GPIO.input(backObstaclePin)):
        print("found surface")
        return True
    return False

def detectObstacle(Pin):
    if (0 == GPIO.input(Pin)):
        return True
    return False

def starting():
    print("starting")
    #start the brush
    brushMotor.setSpeed(baseSpeed)
    brushMotor.run(Adafruit_MotorHAT.FORWARD);
    #loop until edge found
    leftMotor.setSpeed(slowSpeed)
    leftMotor.run(Adafruit_MotorHAT.FORWARD);
    rightMotor.setSpeed(baseSpeed)
    rightMotor.run(Adafruit_MotorHAT.FORWARD);
    while detectObstacle(rightObstaclePin):
        time.sleep(.05)
        
def cleaningSurfaceTurn():
    print("Cleaning Surface Turn")
    leftMotor.setSpeed(0)
    leftMotor.run(Adafruit_MotorHAT.FORWARD);
    rightMotor.setSpeed(0)
    rightMotor.run(Adafruit_MotorHAT.FORWARD);
    if(not detectObstacle(rightObstaclePin)):
        #right sensor is off the edge
        leftMotor.setSpeed(slowSpeed)
        leftMotor.run(Adafruit_MotorHAT.BACKWARD);
        while detectObstacle(backObstaclePin):
            time.sleep(.05)        
    else:
        #left sensor is off the edge
        rightMotor.setSpeed(slowSpeed)
        rightMotor.run(Adafruit_MotorHAT.BACKWARD);
        while detectObstacle(backObstaclePin):
            time.sleep(.05)
    
def cleaningTableSurface():
    print("Cleaning Table Surface")
    leftMotor.setSpeed(baseSpeed)
    leftMotor.run(Adafruit_MotorHAT.FORWARD);
    rightMotor.setSpeed(baseSpeed)
    rightMotor.run(Adafruit_MotorHAT.FORWARD);
    #have a 1 in x chance of turning
    if(random.randint(0, 50) == 0):
        if(random.randint(0, 1) == 0):
            #turn right
            leftMotor.setSpeed(0)
            leftMotor.run(Adafruit_MotorHAT.FORWARD);
        else:
            #turn left
            rightMotor.setSpeed(0)
            rightMotor.run(Adafruit_MotorHAT.FORWARD);
        time.sleep(.1)
    time.sleep(.05)
    
def cleaningTableEdges():
    print("Cleaning Table Edges")
    #wobble back and forth keeping the sensor on the edge of the table
    
def cleaningEdgesTurn():
    print("Cleaning Table Turn")
    #back up a bit, turn a bit and head forward until edge detected.

def loop():
	while True:
            if(vacuumState.state == "waiting for surface"):
                if(detectSurface()):
                    vacuumState.surfaceDetected()
            elif(vacuumState.state == "starting"):
                starting()
                vacuumState.edgeDetected()
            elif(vacuumState.state == "cleaning surface turn"):
                cleaningSurfaceTurn()
                vacuumState.turnCompleted()
            elif(vacuumState.state == "cleaning table surface"):
                cleaningTableSurface()
                vacuumState.edgeDetected()
                
            time.sleep(.02)


def destroy():
	GPIO.cleanup()                     # Release resource

if __name__ == '__main__':     # Program start from here
    setup()
    try:
        vacuumState.poweredOn()
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
