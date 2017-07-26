from master.fsm import StateMachine
from states.select_item import SelectItem
from states.plan_pick_item import PlanPickItem
from states.plan_inspection_station import PlanInspectionStation
from states.plan_pick_only import PlanPickOnly
from states.plan_place_box import PlanPlaceBox
from states.plan_place_shelf import PlanPlaceShelf
from states.exec_route import ExecRoute
from states.check_item import CheckItem
from states.check_route import CheckRoute
from states.check_select_item import CheckSelectItem
from states.plan_view_location import PlanViewLocation
from states.capture_photo_bin import CapturePhotoBin
from states.capture_photo_box import CapturePhotoBox
from states.capture_photo_inspect import CapturePhotoInspect
from states.segment_photo import SegmentPhoto
from states.recognize_photo import RecognizePhoto
from states.evaluate_vacuum_grasp_pick import EvaluateVacuumGraspPick
from states.evaluate_pinch_grasp_pick import EvaluatePinchGraspPick
from states.evaluate_placement import EvaluatePlacement
from states.read_scales_pick import ReadScalesPick
from states.inspect_item import InspectItem
from states.power_cycle_cameras import PowerCycleCameras
from states.detect_grab import DetectGrab
from states.replace_exactly import ReplaceExactly

class PickStateMachine(StateMachine):

    def loadStates(self):
        self.add('pvla', PlanViewLocation('pvla', store=self.store))
        self.add('pvlb', PlanViewLocation('pvlb', store=self.store))
        self.add('pvlc', PlanViewLocation('pvlc', store=self.store))
        self.add('pvl', PlanViewLocation('pvl', store=self.store))
        self.add('cpba', CapturePhotoBin('cpba', store=self.store))
        self.add('cpbb', CapturePhotoBin('cpbb', store=self.store))
        self.add('cpbc', CapturePhotoBin('cpbc', store=self.store))
        self.add('cpb', CapturePhotoBin('cpb', store=self.store))
        self.add('cpx', CapturePhotoBox('cpx', store=self.store))
        self.add('cpi', CapturePhotoInspect('cpi', store=self.store))
        self.add('si', SelectItem('si', store=self.store))
        #self.add('ppi', PlanPickItem('ppi', store=self.store))
        self.add('ppo', PlanPickOnly('ppo', store=self.store))
        self.add('pis', PlanInspectionStation('pis', store=self.store))
        self.add('cr1', CheckRoute('cr1', store=self.store))
        self.add('er1', ExecRoute('er1', store=self.store))
        self.add('ppb', PlanPlaceBox('ppb', store=self.store))
        self.add('pps', PlanPlaceShelf('pps', store=self.store))
        self.add('er2', ExecRoute('er2', store=self.store))
        self.add('ci', CheckItem('ci', store=self.store), endState=1)
        self.add('csi', CheckSelectItem('csi', store=self.store))
        self.add('cr2', CheckRoute('cr2', store=self.store))
        self.add('cr3', CheckRoute('cr3', store=self.store))
        self.add('cr4', CheckRoute('cr4', store=self.store))
        self.add('cr5', CheckRoute('cr5', store=self.store))
        self.add('cr6', CheckRoute('cr6', store=self.store))
        self.add('cr7', CheckRoute('cr7', store=self.store))
        self.add('cr8', CheckRoute('cr8', store=self.store))
        self.add('er3', ExecRoute('er3', store=self.store))
        self.add('er4', ExecRoute('er4', store=self.store))
        self.add('er5', ExecRoute('er5', store=self.store))
        self.add('er6', ExecRoute('er6', store=self.store))
        self.add('er7', ExecRoute('er7', store=self.store))
        self.add('er8', ExecRoute('er8', store=self.store))
        self.add('sp1', SegmentPhoto('sp1', store=self.store))
        self.add('sp2', SegmentPhoto('sp2', store=self.store))
        self.add('sp3', SegmentPhoto('sp3', store=self.store))
        self.add('rp1', RecognizePhoto('rp1', store=self.store))
        self.add('rp2', RecognizePhoto('rp2', store=self.store))
        self.add('rp3', RecognizePhoto('rp3', store=self.store))
        self.add('egvp', EvaluateVacuumGraspPick('egvp', store=self.store))
        self.add('egpp', EvaluatePinchGraspPick('egpp', store=self.store))
        self.add('ii', InspectItem('ii', store=self.store))
        self.add('rs2', ReadScalesPick('rs2', store=self.store))
        self.add('rs1', ReadScalesPick('rs1', store=self.store))
        self.add('rs3', ReadScalesPick('rs3', store=self.store))
        self.add('ep', EvaluatePlacement('ep', store=self.store))
        self.add('pcca', PowerCycleCameras('pcca', store=self.store))
        self.add('pccb', PowerCycleCameras('pccb', store=self.store))
        self.add('pccc', PowerCycleCameras('pccc', store=self.store))
        self.add('pcc', PowerCycleCameras('pcc', store=self.store))
        self.add('pccx', PowerCycleCameras('pccx', store=self.store))
        self.add('pcci', PowerCycleCameras('pcci', store=self.store))
        self.add('dg', DetectGrab('dg', store=self.store))
        self.add('re', ReplaceExactly('re', store=self.store))

    def getStartState(self):
        return 'pvla'

    def setupOther(self):
        self.store.put('/robot/failed_grasps', [])
        self.store.put('/status', [])
        self.store.put('/robot/target_locations', ['binA', 'binB', 'binC'])

    def setupTransitions(self):
        #initial look. only needs to run once...
        self.setTransition('pvla', 'er1', ['pvla', 'pvlb'], checkState='cr1')
        self.setTransition('cr1', 'er1', ['pvla'])
        self.setTransition('er1', 'cpba', ['pvla', 'er1'])
        self.setTransition('cpba', 'pvlb', ['cpba', 'pcca', 'pvlb'])
        self.setTransition('pcca', 'cpba', ['pcca', 'cpba'])

        self.setTransition('pvlb', 'er2', ['pvlb', 'pvlc'], checkState='cr2')
        self.setTransition('cr2', 'er2', ['pvlb'])
        self.setTransition('er2', 'cpbb', ['pvlb', 'er2'])
        self.setTransition('cpbb', 'pvlc', ['cpbb', 'pccb', 'pvlc'])
        self.setTransition('pccb', 'cpbb', ['pccb', 'cpbb'])

        self.setTransition('pvlc', 'er3', ['pvlc', 'pvla'], checkState='cr3')
        self.setTransition('cr3', 'er3', ['pvlc'])
        self.setTransition('er3', 'cpbc', ['pvlc', 'er3'])
        self.setTransition('cpbc', 'rs2', ['cpbc', 'pccc', 'rs2'])
        self.setTransition('pccc', 'cpbc', ['pccc', 'cpbc'])
        self.setTransition('rs2', 'sp1', ['sp1']) # initial read scales

        #then loop to updated pvlXXX and capture
        self.setTransition('sp1', 'rp1', ['sp1', 'pvla', 'rp1'])
        self.setTransition('rp1', 'egvp', ['rp1', 'pvla', 'egvp'])
        self.setTransition('egvp', 'si', ['egvp', 'pvla', 'si'])

        self.setTransition('si', 'ppo', ['pvla', 'pvla'], checkState='csi')
        self.setTransition('csi', 'ppo', ['si'])

        self.setTransition('ppo', 'er4', ['ppo', 'si'], checkState='cr4')
        self.setTransition('cr4', 'er4', ['ppo'])
        self.setTransition('er4', 'rs1', ['ppo', 'er4'])

        self.setTransition('rs1', 'dg', ['dg'])
        self.setTransition('dg', 'pis', ['si', 'si'])
        self.setTransition('pis', 'er7', ['pis', 'si'], checkState='cr7')
        self.setTransition('cr7', 'er7', ['pis'])
        self.setTransition('er7', 'cpi', ['pis', 'er7'])

        self.setTransition('cpi', 'sp2', ['cpi', 'pcci', 'ii'])
        self.setTransition('pcci', 'cpi', ['pcci', 'cpi'])
        self.setTransition('sp2', 'rp2', ['sp2', 'cpi', 'rp2'])
        self.setTransition('rp2', 'ii', ['rp2', 'cpi', 'ii'])
        self.setTransition('ii', 'ep', ['ii', 'si', 're'])

        #wrong item, put back
        self.setTransition('re', 'pps', ['pps']) #TODO
        self.setTransition('pps', 'er8', ['pps', 're'], checkState='cr8') #TODO
        self.setTransition('cr8', 'er8', ['pps'])
        self.setTransition('er8', 'rs3', ['pps', 'er8'])
        self.setTransition('rs3', 'si', ['si'])

        #correct item, putting in box
        self.setTransition('ep', 'ppb', ['ep', 'cpi', 'ii'])
        self.setTransition('ppb', 'er5', ['ppb', 'ep'], checkState='cr5')
        self.setTransition('cr5', 'er5', ['ppb'])
        self.setTransition('er5', 'cpx', ['ppb', 'er5'])

        self.setTransition('cpx', 'ci', ['cpx'])
        #self.setTransition('sp3', 'rp3', ['rp3'])
        #self.setTransition('rp3', 'ci', ['ci'])

        self.setTransition('ci', 'pvl', ['ci'])
        self.setTransition('pvl', 'er6', ['pvl', 'sp1'], checkState='cr6')
        self.setTransition('cr6', 'er6', ['pvl'])
        self.setTransition('er6', 'cpb', ['pvl', 'er6'])
        self.setTransition('cpb', 'sp1', ['cpb', 'pcc', 'sp1'])
        self.setTransition('pcc', 'cpb', ['pcc', 'cpb'])

    def isDone(self):
        return False
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
