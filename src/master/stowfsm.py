from master.fsm import StateMachine
from states.find_all import FindAll
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_stow_grab import PlanStowGrab
from states.plan_place_shelf import PlanPlaceShelf
from states.exec_route import ExecRoute
from states.check_item import CheckItem
from states.check_select_item import CheckSelectItem
from states.check_route import CheckRoute
from states.plan_view_location import PlanViewLocation
from states.capture_photo import CapturePhoto
from states.segment_photo import SegmentPhoto
from states.recognize_photo import RecognizePhoto
from states.evaluate_grasp import EvaluateGrasp
from states.evaluate_placement import EvaluatePlacement
from states.read_scales import ReadScales

class StowStateMachine(StateMachine):

    def loadStates(self):
        #self.add('fa', FindAll('fa', store=self.store))
        self.add('cp', CapturePhoto('cp', store=self.store))
        self.add('sp', SegmentPhoto('sp', store=self.store))
        self.add('rp', RecognizePhoto('rp', store=self.store))
        self.add('eg', EvaluateGrasp('eg', store=self.store))
        self.add('si', SelectItem('si', store=self.store))
        #self.add('fi', FindItem('fi', store=self.store))
        self.add('psg', PlanStowGrab('psg', store=self.store))
        self.add('pvl', PlanViewLocation('pvl', store=self.store))
        self.add('pps', PlanPlaceShelf('pps', store=self.store))
        self.add('er1', ExecRoute('er1', store=self.store))
        self.add('er2', ExecRoute('er2', store=self.store))
        self.add('er3', ExecRoute('er3', store=self.store))
        self.add('ci', CheckItem('ci', store=self.store), endState=1)
        self.add('csi', CheckSelectItem('csi', store=self.store))
        self.add('cr1', CheckRoute('cr1', store=self.store))
        self.add('cr2', CheckRoute('cr2', store=self.store))
        self.add('cr3', CheckRoute('cr3', store=self.store)) #TODO create check states on the fly?
        self.add('rs', ReadScales('rs', store=self.store))
        self.add('ep', EvaluatePlacement('ep', store=self.store))

    def getStartState(self):
        #TODO put this action in separate state?
        self.store.put('/robot/target_location', 'stow_tote')
        return 'cp'
        #return 'si'

    def setupTransitions(self):
        self.setTransition('cp', 'sp', 'cp')
        self.setTransition('sp', 'rp', 'sp') #TODO make handler for these failed states (hardware)
        self.setTransition('rp', 'eg', 'rp')
        self.setTransition('eg', 'si', 'eg')
        self.setTransition('si', 'psg', 'si', checkState='csi')
        self.setTransition('csi', 'psg', 'si')
        self.setTransition('psg', 'er1', 'si', checkState='cr1')
        self.setTransition('cr1', 'er1', 'psg')
        self.setTransition('er1', 'pvl', 'psg')
        self.setTransition('pvl', 'er3', 'si', checkState='cr3') #TODO similar for failed route planning
        self.setTransition('er3', 'ep', 'pvl')
        self.setTransition('ep', 'pps', 'ep') #TODO need failure analysis
        self.setTransition('pps', 'er2', 'si', checkState='cr2')
        self.setTransition('cr2', 'er2', 'pps')
        self.setTransition('er2', 'rs', 'pps')
        self.setTransition('rs', 'ci', 'ci') #TODO handler for failed read scales
        self.setTransition('ci', 'pvl', 'ci') #TODO handler for failed check item
        self.setTransition(' 

    def isDone(self):
        #if all items stowed, all their point values are 0. Need to re-write
        self.value = 0
        for i, n in self.store.get('/item/').items():
            for k in n['items']:
                self.points = self.store.get('/item/'+k+'/point_value')
                self.value+=self.points
        return (self.value==0)

#################################################################################
def runStowFSM():
    stow = StowStateMachine()
    stow.loadStates()
    stow.setupTransitions()

    # initialize workcell
    from master import workcell
    workcell.setup_stow(
        stow.store,
        location='db/item_location_file_stow.json'
    )

    #simulate for now
    stow.store.put('/simulate/robot_motion', True)
    stow.store.put('/simulate/object_detection', True)
    stow.store.put('/simulate/cameras', True)

    #number = 10
    #for _ in range(number): pick.runOrdered('si')
    stow.setCurrentState(stow.getStartState())
    stow.runStep()
    while(not stow.isDone()): stow.runOrdered(stow.getCurrentState())

if __name__ == '__main__':
    runStowFSM()
