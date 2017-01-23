class State:
    def __init__(self, name):
        self.name = name.upper()
    def __str__(self):
        return self.name

class StateMachine:
    def __init__(self, config={}, initial=None, events=None, callbacks=None, final=None):
        config = dict(config)
        events = []
        self.callbacks = {}
        self.initState = initial
        self.finState = None
        self.current = None

    def setCurrentState(self, name)
        self.current = name.upper()

    def isCurrentState(self, name)
        return self.current == name

    def _initialize(self, config)
        initState = config['initial'] if 'initial' in config else None
        self.finState = config['final'] if 'final' in config else None
        events = config['events'] if 'events' in config else []
        callbacks = config['callbacks'] if 'callbacks' in config else {}
       
        tmap = {}
        self._map = tmap

    def add(self, name, event, endState=0)
        name = name.upper()
        self.events[name] = event
        if endState:
            self.finState.append(name)

    def setInit(self, name)
        self.initState = name.upper()  

    def runAll(self, inputs)
        for i in inputs:
            self.current = self.current.next(i)
            self.current.run()
    
class PickObject(State):
    def run(self):
        print("Choosing object from database")
        #something
   
    def next(self, input):
        if input == 
