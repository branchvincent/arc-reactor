from pensive.core import Store
from pensive.client import PensiveClient

class State:
    def __init__(self, name, store=None):
        self.name = name.upper()
        self.store = store or PensiveClient().default()
    def __str__(self):
        return self.name
    def run(self):
        raise NotImplementedError

class StateMachine:
    def __init__(self, events=None, callbacks=None, finStates=None):
        self.events = {}
        self.callbacks = {}
        self.transitions = {}
        self.finStates = {}
        self.pastEvents = {}
        self.current = None

    def setCurrentState(self, name):
        self.current = name.upper()

    def isCurrentState(self, name):
        return self.current == name.upper()

    def getCurrentState(self):
        return self.current

    def add(self, name, event, endState=0):
        name = name.upper()
        self.events[name] = event
        if endState:
            self.finStates[name] = event

    def runCurrent(self):
        self.events[self.current].run()
        self.pastEvents[self.current] = self.events[self.current]

    def runAll(self):
        for i, n in self.events.items():
            self.setCurrentState(i)
            self.runCurrent()
            if(self.current in self.finStates):
                return
                
    def getAllPastEvents(self):
        return self.pastEvents.keys()

    def addTransition(self, name, nameNext):
        self.transitions[name.upper()] = nameNext.upper()

    def runOrdered(self, nameInit):
        if not self.finStates:
            raise RuntimeError("Need to define a final state") 

        self.setCurrentState(nameInit)
        for _ in range(len(self.transitions)+1):
            self.runCurrent()
            print "currently running ", self.getCurrentState()
            if self.current in self.finStates:
                print "Finished running all states."
                return
            else:
                self.setCurrentState(self.transitions[self.getCurrentState()])

