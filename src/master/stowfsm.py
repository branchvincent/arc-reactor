from master.fsm import StateMachine
from states.select_item import SelectItem
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
from states.evaluate_placement_amnesty import EvaluatePlacementAmnesty
from states.read_scales_pick import ReadScalesPick
from states.inspect_item import InspectItem
from states.detect_grab import DetectGrab
from states.power_cycle_cameras import PowerCycleCameras
from states.plan_place_box import PlanPlaceBox
from states.capture_photo_amnesty import CapturePhotoAmnesty
from states.plan_pick_lift import PlanPickLift

class StowStateMachine(StateMachine):

    def loadStates(self):
        self.add('cps', CapturePhotoStow('cps', store=self.store))
        self.add('cpi', CapturePhotoInspect('cpi', store=self.store))
        self.add('cpb', CapturePhotoBin('cpb', store=self.store))
        self.add('cpa', CapturePhotoAmnesty('cpa', store=self.store))
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
        self.add('ppl', PlanPickLift('ppl', store=self.store))
        self.add('pis', PlanInspectionStation('pis', store=self.store))
        self.add('pvl', PlanViewLocation('pvl', store=self.store))
        self.add('pvl2', PlanViewLocation('pvl2', store=self.store))
        self.add('pps', PlanPlaceShelf('pps', store=self.store))
        self.add('ppb', PlanPlaceBox('ppb', store=self.store)) #for amnesty tote
        self.add('er1', ExecRoute('er1', store=self.store))
        self.add('er11', ExecRoute('er11', store=self.store))
        self.add('er2', ExecRoute('er2', store=self.store))
        self.add('er3', ExecRoute('er3', store=self.store))
        self.add('er4', ExecRoute('er4', store=self.store))
        self.add('er5', ExecRoute('er5', store=self.store))
        self.add('er6', ExecRoute('er6', store=self.store))
        self.add('ci', CheckItem('ci', store=self.store), endState=1)
        self.add('ci2', CheckItem('ci2', store=self.store), endState=1)
        self.add('csi', CheckSelectItem('csi', store=self.store))
        self.add('cr1', CheckRoute('cr1', store=self.store))
        self.add('cr11', CheckRoute('cr11', store=self.store))
        self.add('cr2', CheckRoute('cr2', store=self.store))
        self.add('cr3', CheckRoute('cr3', store=self.store))
        self.add('cr4', CheckRoute('cr4', store=self.store))
        self.add('cr5', CheckRoute('cr5', store=self.store))
        self.add('cr6', CheckRoute('cr6', store=self.store))
        self.add('rss1', ReadScalesPick('rss1', store=self.store))
        self.add('rss2', ReadScalesPick('rss2', store=self.store))
        self.add('rss', ReadScalesPick('rss', store=self.store))
        self.add('ep', EvaluatePlacement('ep', store=self.store))
        self.add('epa', EvaluatePlacementAmnesty('epa', store=self.store))
        self.add('dg', DetectGrab('dg', store=self.store))
        self.add('pccs', PowerCycleCameras('pccs', store=self.store))
        self.add('pcci', PowerCycleCameras('pcci', store=self.store))
        self.add('pccb', PowerCycleCameras('pccb', store=self.store))
        self.add('pcca', PowerCycleCameras('pcca', store=self.store))


    def getStartState(self):
        return 'cps'

    def setupOther(self):
        self.store.put('/robot/failed_grasps', [])
        self.store.put('/status', [])
        self.store.put('/failure', [])
        self.store.put('/robot/target_view_location', 'stow_tote')

    def setupTransitions(self):
        self.setTransition('cps', 'sp1', ['cps', 'pccs', 'sp1'])
        self.setTransition('pccs', 'cps', ['pccs', 'cps'])
        self.setTransition('sp1', 'rp1', ['sp1', 'cps', 'cps'])
        self.setTransition('rp1', 'rss1', ['rp1', 'cps', 'rss1'])
        self.setTransition('rss1', 'egvs', ['egvs'])

        self.setTransition('egvs', 'si', ['egvs', 'cps', 'si'])
        self.setTransition('si', 'ppo', ['egvs', 'cps'], checkState='csi')
        self.setTransition('csi', 'ppo', ['si'])
        self.setTransition('ppo', 'er1', ['ppo', 'si'], checkState='cr1')
        self.setTransition('cr1', 'er1', ['ppo'])
        self.setTransition('er1', 'rss', ['ppo', 'er1'])

        self.setTransition('rss', 'dg', ['dg']) #if scales fail continue anyway
        self.setTransition('dg', 'pis', ['si', 'si', 'cps']) #detect grab failure means we didn't get the item
        self.setTransition('pis', 'er4', ['pis', 'si'], checkState='cr4')
        self.setTransition('cr4', 'er4', ['pis'])
        self.setTransition('er4', 'cpi', ['pis', 'er4'])

        self.setTransition('cpi', 'sp2', ['cpi', 'pcci', 'ii'])
        self.setTransition('pcci', 'cpi', ['pcci', 'cpi'])
        self.setTransition('sp2', 'rp2', ['sp2', 'epa', 'cpi'])
        self.setTransition('rp2', 'ii', ['rp2', 'cpi', 'ii'])
        self.setTransition('ii', 'ep', ['ii', 'si', 'epa'])

        #put in shelf
        self.setTransition('ep', 'pps', ['ep', 'cpi', 'ii'])
        self.setTransition('pps', 'er2', ['pps', 'ep', 'epa'], checkState='cr2')
        self.setTransition('cr2', 'er2', ['pps'])
        self.setTransition('er2', 'ci', ['pps', 'er2'])

        #put in amnesty tote if unknown
        self.setTransition('epa', 'ppb', ['epa', 'cpi', 'ii'])
        self.setTransition('ppb', 'er5', ['ppb', 'epa'], checkState='cr5')
        self.setTransition('cr5', 'er5', ['ppb'])
        self.setTransition('er5', 'ci2', ['ppb', 'er5'])
        self.setTransition('ci2', 'cps', ['ci2'])

        #move, take photo of bin in shelf, and loop
        self.setTransition('ci', 'pvl', ['ci'])
        self.setTransition('pvl', 'er3', ['pvl', 'cps'], checkState='cr3')
        self.setTransition('cr3', 'er3', ['pvl'])
        self.setTransition('er3', 'cpb', ['pvl', 'er3'])
        self.setTransition('cpb', 'cps', ['cpb', 'pccb', 'cps'])

    def isDone(self):
        return False

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
