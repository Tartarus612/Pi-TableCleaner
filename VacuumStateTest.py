#!/usr/bin/python
#!/home/pi/transitions
#!/usr/local/lib/python3.5/dist-packages
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
        self.surfaceEdgesNeeded = 9
        #number of edge edges to find to be done cleaning
        self.edgeEdgesNeeded = 3

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


def loop():
    while True:
        print("state = " + vacuumState.state)
        inkey = input()
        if inkey == "a":
            print("poweredOn")
            vacuumState.poweredOn()
        elif inkey == "b":
            print("surfaceDetected")
            vacuumState.surfaceDetected()
        elif inkey == "c":
            print("edgeDetected")
            vacuumState.edgeDetected()
        elif inkey == "d":
            print("turnCompleted")
            vacuumState.turnCompleted()
        elif inkey == "e":
            print("pickedUp")
            vacuumState.pickedUp()
        else:
            print("Other input")
			


if __name__ == '__main__':     # Program start from here
    loop()