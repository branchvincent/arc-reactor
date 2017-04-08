from master.fsm import StateMachine
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute
from states.exec_route import ExecRoute
from states.check_item import CheckItem
from states.check_route import CheckRoute
from states.check_select_item import CheckSelectItem
from states.move_shelf import MoveShelf

class PickStateMachine(StateMachine):

    def loadStates(self):
        self.add('si', SelectItem('si', store=self.store))
        self.add('fi', FindItem('fi', store=self.store))
        self.add('pr', PlanRoute('pr', store=self.store))
        self.add('ms', MoveShelf('ms', store=self.store))
        self.add('er', ExecRoute('er', store=self.store))
        self.add('ci', CheckItem('ci', store=self.store), endState=1)
        self.add('csi', CheckSelectItem('csi', store=self.store))
        self.add('cr', CheckRoute('cr', store=self.store))

    def setupTransitions(self):
        self.setTransition('si', 'fi', 'si', '/status/selected_item', '/checkpoint/select_item', checkState='csi')
        self.setTransition('csi', 'fi', 'si', '/status/selected_item')
        self.setTransition('fi', 'pr', 'ms', '/status/selected_item_location')
        self.setTransition('pr', 'er', 'ms', '/status/route_plan', '/checkpoint/plan_route', checkState='cr')
        self.setTransition('cr', 'er', 'pr', '/status/route_plan')
        self.setTransition('er', 'ci', 'fi', '/status/route_exec')
        self.setTransition('ms', 'fi', 'fi', '/status/shelf_move')
        self.setTransition('ci', 'si', 'fi', '/status/item_picked')

    def isDone(self):
        #if all items picked, all their point values are 0. Need to re-write
        self.value = 0
        for i, n in self.store.get('/order/').items():
            for k in n['contents']:
                self.points = self.store.get('/item/'+k+'/point_value')
                self.value+=self.points
        return (self.value==0)


#################################################################################
def runPickFSM():
    pick = PickStateMachine()
    pick.loadStates()
    pick.setupTransitions()

    # initialize workcell
    from master import workcell
    workcell.setup_pick(
        pick.store,
        location='db/item_location_file_pick.json',
        order='db/order_file.json'
    )

    #simulate for now
    pick.store.put('/simulate/robot_motion', True)
    pick.store.put('/simulate/object_detection', True)
    pick.store.put('/simulate/cameras', True)

    pick.setCurrentState('si')

    pick.runStep()
    while(not pick.isDone()): pick.runOrdered(pick.getCurrentState())

if __name__ == '__main__':
    runPickFSM()
