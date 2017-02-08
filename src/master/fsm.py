from pensive.core import Store
from pensive.client import PensiveClient

class State():
    def __init__(self, name, store=None):
        self.name = name.upper()
        self.store = store or PensiveClient().default()
    def __str__(self):
        return self.name
    def run(self):
        raise NotImplementedError

class Transition():
    def __init__(self, fromState, toState, altState, condition=None):
        self.fromState = fromState.upper()
        self.toState = toState.upper()
        self.altState = altState.upper()
        self.condition = condition

    def decideTransition(self):
        if self.condition == None:
            return self.toState
        else:
            return (self.toState if self.condition else self.altState)

class StateMachine():
    def __init__(self, events=None, callbacks=None, finStates=None, store=None):
        self.events = {}
        self.callbacks = {}
        self.transitions = {}
        self.finStates = {}
        self.pastEvents = []
        self.current = None
        self.store = store or PensiveClient().default()

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
        self.pastEvents.append(self.current)

    def runAll(self):
        for i, n in self.events.items():
            self.setCurrentState(i)
            self.runCurrent()
            if(self.current in self.finStates):
                return
                
    def getAllPastEvents(self):
        return self.pastEvents

    def setTransition(self, name, nameNext, altNext, condition=None):
        if condition == None:
            self.check = condition
            #need to check if errors occurred in State
        else: self.check = self.store.get(condition)
        self.transitions[name.upper()]=Transition(name, nameNext, altNext, self.check)

    def getTransitions(self):
        return self.transitions

    def runOrdered(self, nameInit):
        if not self.finStates:
            raise RuntimeError("Need to define a final state") 

        self.setCurrentState(nameInit)
        for _ in range(len(self.transitions)+1):
            self.runCurrent()
            #print "currently running ", self.getCurrentState()
            if self.current in self.finStates:
                #print "Finished running all states."
                return
            else:
                self.decideState = self.transitions[self.getCurrentState()].decideTransition()
                #print "decided to go to: ", self.decideState
                self.setCurrentState(self.decideState)

