from master.fsm import StateMachine
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute
from states.exec_route import ExecRoute

class PickStateMachine(StateMachine):

    def loadStates(self):
        self.add('si1', SelectItem('si1'))
        self.add('fi1', FindItem('fi1'))
        self.add('pr1', PlanRoute('pr1'))
        self.add('er1', ExecRoute('er1'), endState=1)

    def setupTransitions(self):
        self.addTransition('si1', 'fi1')
        self.addTransition('fi1', 'pr1')
        self.addTransition('pr1', 'er1')


def runPickFSM():
    pick = PickStateMachine()
    pick.loadStates()
    pick.setupTransitions()
    pick.runOrdered('si1')
    pick.runOrdered('pr1')

def runPickFSM_Eval():
    pick = PickStateMachine()
    pick.loadStates()

if __name__ == '__main__':
    runPickFSM()
