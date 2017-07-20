from master.fsm import StateMachine
from states.select_item import SelectItem
#from states.plan_stow_grab import PlanStowGrab
from states.plan_inspection_station import PlanInspectionStation
from states.plan_pick_only import PlanPickOnly
from states.plan_place_shelf import PlanPlaceShelf
from states.exec_route import ExecRoute
from states.check_item import CheckItem
from states.check_select_item import CheckSelectItem
from states.check_route import CheckRoute
from states.plan_view_location import PlanViewLocation
from states.capture_photo_stow import CapturePhotoStow
from states.capture_photo_inspect import CapturePhotoInspect
from states.capture_photo_bin import CapturePhotoBin
from states.segment_photo import SegmentPhoto
from states.recognize_photo import RecognizePhoto
from states.evaluate_vacuum_grasp_stow import EvaluateVacuumGraspStow
from states.evaluate_pinch_grasp_stow import EvaluatePinchGraspStow
from states.evaluate_placement import EvaluatePlacement
from states.read_scales import ReadScales
from states.inspect_item import InspectItem
from states.detect_grab import DetectGrab

class StowStateMachine(StateMachine):

    def loadStates(self):
        self.add('cps', CapturePhotoStow('cps', store=self.store))
        self.add('cpi', CapturePhotoInspect('cpi', store=self.store))
        self.add('cpb', CapturePhotoBin('cpb', store=self.store))
        self.add('sp1', SegmentPhoto('sp1', store=self.store))
        self.add('sp2', SegmentPhoto('sp2', store=self.store))
        self.add('sp3', SegmentPhoto('sp3', store=self.store))
        self.add('rp1', RecognizePhoto('rp1', store=self.store))
        self.add('rp2', RecognizePhoto('rp2', store=self.store))
        self.add('rp3', RecognizePhoto('rp3', store=self.store))
        self.add('ii', InspectItem('ii', store=self.store))
        self.add('egvs', EvaluateVacuumGraspStow('egvs', store=self.store))
        self.add('egps', EvaluatePinchGraspStow('egps', store=self.store))
        self.add('si', SelectItem('si', store=self.store))
        #self.add('psg', PlanStowGrab('psg', store=self.store))
        self.add('ppo', PlanPickOnly('ppo', store=self.store))
        self.add('pis', PlanInspectionStation('pis', store=self.store))
        self.add('pvl', PlanViewLocation('pvl', store=self.store))
        self.add('pps', PlanPlaceShelf('pps', store=self.store))
        self.add('er1', ExecRoute('er1', store=self.store))
        self.add('er2', ExecRoute('er2', store=self.store))
        self.add('er3', ExecRoute('er3', store=self.store))
        self.add('er4', ExecRoute('er3', store=self.store))
        self.add('ci', CheckItem('ci', store=self.store), endState=1)
        self.add('csi', CheckSelectItem('csi', store=self.store))
        self.add('cr1', CheckRoute('cr1', store=self.store))
        self.add('cr2', CheckRoute('cr2', store=self.store))
        self.add('cr3', CheckRoute('cr3', store=self.store)) #TODO create check states on the fly?
        self.add('cr4', CheckRoute('cr2', store=self.store))
        self.add('rs1', ReadScales('rs1', store=self.store))
        self.add('rs', ReadScales('rs', store=self.store))
        self.add('ep', EvaluatePlacement('ep', store=self.store))
        self.add('dg', DetectGrab('dg', store=self.store))


    def getStartState(self):
        return 'cps'

    def setupOther(self):
        self.store.put('/robot/failed_grasps', [])
        self.store.put('/robot/target_view_location', 'stow_tote')

    def setupTransitions(self):
        self.setTransition('cps', 'sp1', ['sp1']) 
        self.setTransition('sp1', 'rp1', ['sp1', 'cps', 'rp1']) 
        self.setTransition('rp1', 'rs1', ['rp1', 'cps', 'rs1'])
        self.setTransition('rs1', 'egvs', ['egvs'])

        self.setTransition('egvs', 'si', ['egvs', 'cps', 'si'])
        self.setTransition('si', 'ppo', ['si'], checkState='csi') #si should never fail..
        self.setTransition('csi', 'ppo', ['si'])
        self.setTransition('ppo', 'er1', ['si'], checkState='cr1')
        #self.setTransition('psg', 'er1', ['si'], checkState='cr1') #if plan fails, redo SI and mark failure
        self.setTransition('cr1', 'er1', ['ppo'])
        self.setTransition('er1', 'rs', ['ppo', 'er1'])

        self.setTransition('rs', 'dg', ['dg']) #if scales fail continue anyway
        self.setTransition('dg', 'pis', ['si', 'si']) #detect grab failure means we didn't get the item
        self.setTransition('pis', 'er4', ['si'], checkState='cr4')
        self.setTransition('cr4', 'er4', ['pis'])
        self.setTransition('er4', 'cpi', ['pis', 'er4'])

        self.setTransition('cpi', 'sp2', ['ii'])
        self.setTransition('sp2', 'rp2', ['sp2', 'cpi', 'rp2'])
        self.setTransition('rp2', 'ii', ['rp2', 'cpi', 'ii'])
        self.setTransition('ii', 'ep', ['ii', 'si'])

        self.setTransition('ep', 'pps', ['ep'])
        self.setTransition('pps', 'er2', ['si'], checkState='cr2')
        self.setTransition('cr2', 'er2', ['pps'])
        self.setTransition('er2', 'ci', ['pps', 'er2'])
        self.setTransition('ci', 'pvl', ['ci']) 
        self.setTransition('pvl', 'er3', ['pvl', 'cps'], checkState='cr3')
        self.setTransition('cr3', 'er3', ['pvl'])
        self.setTransition('er3', 'cpb', ['pvl', 'er3'])
        self.setTransition('cpb', 'cps', ['cpb']) #TODO maybe revise which photos are segmented at the end

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
