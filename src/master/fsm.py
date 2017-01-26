from pensive.core import Store
from pensive.client import PensiveClient

class State:
    def __init__(self, name):
        self.name = name.upper()
        self.store = PensiveClient().default()
    def __str__(self):
        return self.name
    def run(self):
        raise NotImplementedError

class StateMachine:
    def __init__(self, config={}, initial=None, events=None, callbacks=None, final=None):
        config = dict(config)
        events = []
        self.callbacks = {}
        self.initState = initial
        self.finStates = None
        self.current = None

    def setCurrentState(self, name):
        self.current = name.upper()

    def isCurrentState(self, name):
        return self.current == name

    def add(self, name, event, endState=0):
        name = name.upper()
        self.events[name] = event
        if endState:
            self.finStates.append(name)

    def setInit(self, name):
        self.initState = name.upper()  

##############################################
def runTest():
    fsm = StateMachine()
#    fsm.add('SelectItem', 

if __name__ == '__main__':
    runTest()
    
