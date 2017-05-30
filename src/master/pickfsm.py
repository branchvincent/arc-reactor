from master.fsm import StateMachine
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_pick_item import PlanPickItem
from states.plan_place_box import PlanPlaceBox
from states.exec_route import ExecRoute
from states.check_item import CheckItem
from states.check_route import CheckRoute
from states.check_select_item import CheckSelectItem
from states.plan_view_location import PlanViewLocation

class PickStateMachine(StateMachine):

    def loadStates(self):
        self.add('si', SelectItem('si', store=self.store))
        self.add('fi', FindItem('fi', store=self.store))
        self.add('ppi', PlanPickItem('ppi', store=self.store))
        self.add('cr1', CheckRoute('cr1', store=self.store))
        self.add('pvl', PlanViewLocation('pvl', store=self.store))
        self.add('er1', ExecRoute('er1', store=self.store))
        self.add('ppb', PlanPlaceBox('ppb', store=self.store))
        self.add('er2', ExecRoute('er2', store=self.store))
        self.add('ci', CheckItem('ci', store=self.store), endState=1)
        self.add('csi', CheckSelectItem('csi', store=self.store))
        self.add('cr2', CheckRoute('cr2', store=self.store))
        self.add('cr3', CheckRoute('cr3', store=self.store))
        self.add('er3', ExecRoute('er3', store=self.store))

    def getStartState(self):
        return 'si'

    def setupTransitions(self):
        self.setTransition('si', 'fi', 'si', checkState='csi')
        self.setTransition('csi', 'fi', 'si')
        self.setTransition('fi', 'ppi', 'csi')
        self.setTransition('ppi', 'er1', 'fi', checkState='cr1')
        self.setTransition('cr1', 'er1', 'ppi')
        self.setTransition('er1', 'ppb', 'fi')
        self.setTransition('ppb', 'er2', 'fi', checkState='cr2')
        self.setTransition('cr2', 'er2', 'ppb')
        self.setTransition('er2', 'ci', 'fi')
        self.setTransition('ci', 'si', 'fi')

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
