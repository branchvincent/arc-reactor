from pensive.core import Store
from pensive.client import PensiveClient

import logging; logger = logging.getLogger(__name__)

class State():
    def __init__(self, name, store=None):
        self.name = name.upper()
        self.store = store or PensiveClient().default()
    def __str__(self):
        return self.name
    def run(self):
        raise NotImplementedError

class Transition():
    def __init__(self, fromState, toState, altState, condition=None, checkpoint=None, store=None, checkState=None):
        self.fromState = fromState.upper()
        self.toState = toState.upper()
        self.altState = altState.upper()
        self.checkState = checkState
        self.condition = condition
        self.checkpoint = checkpoint
        self.store = store or PensiveClient().default()

    def decideTransition(self):
        if self.checkpoint is not None:
            if self.store.get(self.checkpoint):
                print "got checkpoint"
                return self.checkState.upper()
            else:
                pass
        if self.condition == None:
            print "no condition"
            return self.toState
        else:
            return (self.toState if self.store.get(self.condition) else self.altState)

class StateMachine():
    def __init__(self, events=None, callbacks=None, finStates=None, store=None):
        self.events = {}
        self.callbacks = {}
        self.transitions = {}
        self.finStates = {}
        self.pastEvents = []
        self.pastStorage = []
        self.current = None
        self.store = store or PensiveClient().default()
        self.removeHistory()

    def removeHistory(self):
        for i in PensiveClient().index():
            if i is not None:
                PensiveClient().delete(i)

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
        print "currently running ", self.getCurrentState()
        logger.info("Current running the state {} ".format(self.getCurrentState()))
        self.pastStorage.append(PensiveClient().create("history"+self.getCurrentState(), parent=self.store.get_instance()))
        self.events[self.current].run()
        self.pastEvents.append(self.current)
        
    def getLast(self):
        lastEvent = self.pastEvents.pop()
        self.pastStore = self.pastStorage.pop()
        print "going back to ", lastEvent
        logger.info("Going back one state to {} ".format(lastEvent))
        return lastEvent

    def runAll(self):
        for i, n in self.events.items():
            self.setCurrentState(i)
            self.runCurrent()
            if(self.current in self.finStates):
                return

    def getAllPastEvents(self):
        return self.pastEvents

    def setTransition(self, name, nameNext, altNext, condition=None, checkpoint=None, checkState=None):
        self.transitions[name.upper()]=Transition(name, nameNext, altNext, condition, checkpoint, self.store, checkState)

    def getTransitions(self):
        return self.transitions

    def runOrdered(self, nameInit):
        if not self.finStates:
            raise RuntimeError("Need to define a final state")

        self.setCurrentState(nameInit)

        while self.current not in self.finStates:
            self.runCurrent()
            self.decideState = self.transitions[self.getCurrentState()].decideTransition()
            self.setCurrentState(self.decideState)

        self.runCurrent()
        self.decideState = self.transitions[self.getCurrentState()].decideTransition()
        self.setCurrentState(self.decideState)

    def runStep(self):
        if not self.getCurrentState():
            raise RuntimeError("Need to define initial state")
        if not self.finStates:
            raise RuntimeError("Need to define a final state")

        self.runCurrent()
        self.decideState = self.transitions[self.getCurrentState()].decideTransition()
        self.setCurrentState(self.decideState)  

    def backStep(self):
        if not self.getCurrentState():
            raise RuntimeError("Not in a state. Cannot go back")
        self.setCurrentState(self.getLast())
        print "rewriting db from ", self.pastStore.get_instance()
        print "selected_item in this store is ", self.pastStore.get('/robot/selected_item')
        print "overwriting the store ", self.store.get_instance()
        self.store.put('', self.pastStore.get())
        PensiveClient().delete(self.pastStore.get_instance())

    def isDone(self):
        raise NotImplementedError

