from pensive.core import Store
from pensive.client import PensiveClient
from subprocess import Popen
import inspect
from tornado.ioloop import IOLoop
from tornado import gen
import time

import logging; logger = logging.getLogger(__name__)

class State():
    def __init__(self, name, store=None):
        self.fullname = name
        self.name = name.upper()
        self.store = store or PensiveClient().default()
    def __str__(self):
        return self.name
    def setFullName(self, fullname):
        self.fullname = fullname
    def getFullName(self):
        return self.fullname
    def run(self):
        raise NotImplementedError
    def setOutcome(self, outcome):
        self.outcome = outcome
        self.store.put('/outcome/'+self.getFullName(), outcome)

class Transition():
    def __init__(self, fromState, toState, altState, store=None, checkState=None):
        self.fromState = fromState.upper()
        self.toState = toState.upper()
        self.altState = altState.upper()
        self.checkState = checkState
        self.outcome = '/outcome/'+fromState
        self.checkpoint = '/checkpoint/'+self.findCheckpoint(fromState)
        self.store = store or PensiveClient().default()

    def findCheckpoint(self, fromState):
        if fromState.startswith("p"):
            return "plan_route"
        elif fromState.startswith("si"):
            return "select_item"
        else: return fromState

    def decideTransition(self):
        if not self.store.get(self.outcome, False): #if nothing set, assume failed
            print "Failed, going back, outcome was ", self.outcome
            logger.info('State failed, going to alternative state')
            return self.altState
        if self.store.get(self.checkpoint, False):
            print "got checkpoint"
            logger.info('Checkpoint after current state is called')
            if self.checkState is None:
                raise RuntimeError("Checkpoint asked for, but state is non-existent")
            return self.checkState.upper()
        else: 
            return self.toState

class StateMachine():
    def __init__(self, events=None, transitions=None, finStates=None, pastEvents=None, pastStorage=None, currentState=None, store=None):
        self.events = {}
        self.transitions = {}
        self.finStates = {}
        self.pastEvents = []
        self.pastStorage = []
        self.current = currentState
        self.store = store or PensiveClient().default()
        #self.removeHistory()
        self.setupFlag()
        self.p = None

    def getStartState(self):
        raise NotImplementedError

    def getStore(self):
        return self.store

    def setupFlag(self):
        self.store.put('/robot/stop_flag', False)

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

    def getFinalStates(self):
        return self.finStates

    def getTransitions(self):
        return self.transitions

    def getEvents(self):
        return self.events

    def getPastEvents(self):
        return self.pastEvents

    def getPastStorage(self):
        return self.pastStorage

    def add(self, name, event, endState=0):
        name = name.upper()
        self.events[name] = event
        if endState:
            self.finStates[name] = event

    def setFinal(self, name):
        self.finStates[name] = self.events[name]

    def stop(self):
        self.backStep()
        self.store.put('/robot/stop_flag', True)
        if(self.p is not None):
            self.p.kill()

    def isStateDone(self):
        return not (self.p.poll() is None)

    def runCurrent(self):
        print "currently running ", self.getCurrentState()
        self.events[self.getCurrentState()].setOutcome(False)
        logger.info("Current running the state {} ".format(self.getCurrentState()))
        history_name = self.getCurrentState()+str(len(self.pastStorage))
        if history_name in PensiveClient().index():
            PensiveClient().delete(history_name)
        histStore = PensiveClient().create(history_name, parent=self.store.get_instance())

        whoiam = inspect.getmodule(self.events[self.current]).__name__
        self.p = Popen(['./reactor', 'shell', '-m', whoiam, self.getCurrentState().lower()])
        self.p.wait()

        self.pastEvents.append(self.current)
        self.pastStorage.append(histStore)
        
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

    def setTransition(self, name, nameNext, altNext, checkState=None):
        self.transitions[name.upper()]=Transition(name, nameNext, altNext, self.store, checkState)

    def getTransitions(self):
        return self.transitions

    def runOrdered(self, nameInit):
        if not self.finStates:
            raise RuntimeError("Need to define a final state")

        self.setCurrentState(nameInit)
        #self.removeHistory() #TODO change this for runAll application?
        
        while (self.current not in self.finStates) and (not self.store.get('/robot/stop_flag', False)):
            self.runStep()

        if (not self.store.get('/robot/stop_flag', False)):
            self.runStep()

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
        logger.info("Re-writing the db from {}".format(self.pastStore.get_instance()))
        self.store.put('', self.pastStore.get())
        PensiveClient().delete(self.pastStore.get_instance())

    def isDone(self):
        raise NotImplementedError

