from master.fsm import StateMachine
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute
from states.exec_route import ExecRoute

class PickStateMachine(StateMachine):

    def loadStates(self):
        self.add('si', SelectItem('si'))
        self.add('fi', FindItem('fi'))
        self.add('pr', PlanRoute('pr'))
        self.add('er', ExecRoute('er'), endState=1)

    def setupTransitions(self):
        self.setTransition('si', 'fi', 'si', '/status/selected_item')
        self.setTransition('fi', 'pr', 'ms', '/status/selected_item_location')
        self.setTransition('pr', 'er', 'ms', '/status/route_plan')
        self.setTransition('er', 'fi', 'pr', '/status/route_exec')
        self.setTransition('ms', 'fi', 'fi', '/status/shelf_move')

#################################################################################
def runPickFSM():
    pick = PickStateMachine()
    pick.loadStates()
    pick.setupTransitions()
    pick.runOrdered('si')

def runPickFSM_Eval():
    pick = PickStateMachine()
    pick.loadStates()

if __name__ == '__main__':
    runPickFSM()
