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
from states.capture_photo_bin import CapturePhotoBin
from states.segment_photo import SegmentPhoto
from states.recognize_photo import RecognizePhoto
from states.evaluate_grasp import EvaluateGrasp
from states.evaluate_placement import EvaluatePlacement
from states.read_scales import ReadScales
from states.inspect_item import InspectItem

class PickStateMachine(StateMachine):

    def loadStates(self):
        self.add('pvla', PlanViewLocation('pvla', store=self.store))
        self.add('pvlb', PlanViewLocation('pvlb', store=self.store))
        self.add('pvlc', PlanViewLocation('pvlc', store=self.store))
        self.add('pvl', PlanViewLocation('pvl', store=self.store))
        self.add('cpba', CapturePhotoBin('cpba', store=self.store))
        self.add('cpbb', CapturePhotoBin('cpbb', store=self.store))
        self.add('cpbc', CapturePhotoBin('cpbc', store=self.store))
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
        self.add('cr4', CheckRoute('cr4', store=self.store))
        self.add('cr5', CheckRoute('cr5', store=self.store))
        self.add('er3', ExecRoute('er3', store=self.store))
        self.add('sp1', SegmentPhoto('sp1', store=self.store))
        self.add('sp2', SegmentPhoto('sp2', store=self.store))
        self.add('sp3', SegmentPhoto('sp3', store=self.store))
        self.add('rp1', RecognizePhoto('rp1', store=self.store))
        self.add('rp2', RecognizePhoto('rp2', store=self.store))
        self.add('rp3', RecognizePhoto('rp3', store=self.store))
        self.add('ii', InspectItem('ii', store=self.store))

    def getStartState(self):
        return 'pvla'

    def setupTransitions(self):
        #initial look. only needs to run once...
        self.setTransition('pvla', 'er1', 'pvla', checkState='cr1')
        self.setTransition('cr1', 'er1', 'pvla')
        self.setTransition('er1', 'cpba', 'pvla')
        self.setTransition('cpba', 'pvlb', 'cpba')

        self.setTransition('pvlb', 'er2', 'pvlb', checkState='cr2')
        self.setTransition('cr2', 'er2', 'pvlb')
        self.setTransition('er2', 'cpbb', 'pvlb')
        self.setTransition('cpbb', 'pvlc', 'cpbb')

        self.setTransition('pvlc', 'er3', 'pvlc', checkState='cr3')
        self.setTransition('cr3', 'er3', 'pvlc')
        self.setTransition('er3', 'cpbc', 'pvlc')
        self.setTransition('cpbc', 'sp', 'cpbc')

        #then loop to updated pvlXXX and capture
        self.setTransition('sp', 'rp', 'sp')
        self.setTransition('rp', 'eg', 'rp')
        self.setTransition('eg', 'si', 'eg')

        self.setTransition('si', 'ppi', 'si', checkState='csi')
        self.setTransition('csi', 'ppi', 'si')
        self.setTransition('ppi', 'er4', 'ppi', checkState='cr4')
        self.setTransition('cr4', 'er4', 'ppi')
        self.setTransition('er4', 'ii', 'ppi')
        self.setTransition('ii', 'ep', 'ii')
        self.setTransition('ep', 'ppb', 'ep')
        self.setTransition('ppb', 'er5', 'ppb', checkState='cr5')
        self.setTransition('cr5', 'er5', 'ppb')
        self.setTransition('er5', 'cpx', 'ppb')
        self.setTransition('cpx', 'spx', 'cpx')
        self.setTransition('spx', 'rpx', 'spx')
        self.setTransition('rpx', 'ci', 'rpx') 
        self.setTransition('ci', 'pvl', 'ci')
        self.setTransition('pvl', 'cpbx', 'pvl')
        self.setTransition('cpbx', 'sp', 'cpbx')

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

    pick.setCurrentState(pick.getStartState())

    pick.runStep()
    while(not pick.isDone()): pick.runOrdered(pick.getCurrentState())

if __name__ == '__main__':
    runPickFSM()
