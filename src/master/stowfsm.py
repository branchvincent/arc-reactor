from master.fsm import StateMachine
from states.find_all import FindAll
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute
from states.exec_route import ExecRoute
from states.check_item import CheckItem
from states.check_select_item import CheckSelectItem
from states.check_route import CheckRoute
import json
import cv2
import numpy

class StowStateMachine(StateMachine):

    def loadStates(self):
        self.add('fa', FindAll('fa', store=self.store))
        self.add('si', SelectItem('si', store=self.store))
        self.add('fi', FindItem('fi', store=self.store))
        self.add('pr', PlanRoute('pr', store=self.store))
        self.add('er', ExecRoute('er', store=self.store))
        self.add('ci', CheckItem('ci', store=self.store), endState=1)
        self.add('csi', CheckSelectItem('csi', store=self.store))
        self.add('cr', CheckRoute('cr', store=self.store))

    def setupTransitions(self):
        self.setTransition('fa', 'si', 'fa', '/status/viewed_items')
        self.setTransition('si', 'fi', 'si', '/status/selected_item', '/checkpoint/select_item', checkState='csi')
        self.setTransition('csi', 'fi', 'si', '/status/selected_item')
        self.setTransition('fi', 'pr', 'ms', '/status/selected_item_location')
        self.setTransition('pr', 'er', 'ms', '/status/route_plan', '/checkpoint/plan_route', checkState='cr')
        self.setTransition('cr', 'er', 'pr', '/status/route_plan')
        self.setTransition('er', 'ci', 'fi', '/status/route_exec')
        self.setTransition('ms', 'fi', 'fi', '/status/shelf_move')
        self.setTransition('ci', 'si', 'fi', '/status/item_picked')

    def loadLocationFile(self, file_location):
        with open(file_location) as data_file:
            self.location_db = json.load(data_file)

        # copied from PickStateMachine for testing only
        for (i, (k, v)) in enumerate(self.location_db['item'].items()):
            self.store.put(['item', k, 'location'], v['location'])
            self.points = 10
            self.store.put('/item/'+k+'/point_value', self.points)
            print "item ", k, " is valued at ", self.points, "in", v['location']

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
    with open('test/master/test_021317_2.json') as data_file:
        initial_db = json.load(data_file)
    stow.store.put('', initial_db)
    stow.loadLocationFile('test/master/location_test.json')
    stow.store.put('/robot/task', 'stow')
    #simulate for now
    stow.store.put('/simulate/robot_motion', True)
    stow.store.put('/simulate/object_detection', True)
    #image = cv2.imread('test/camera1.png')
    #pick.store.put('/camera/camera1/color_image', image)
    #pc = numpy.load('test/camera1_pc.npy')
    #pick.store.put('/camera/camera1/point_cloud', pc)
    #number = 10
    #for _ in range(number): pick.runOrdered('si')
    stow.setCurrentState('si')
    stow.runStep()
    while(not stow.isDone()): stow.runOrdered(stow.getCurrentState())

if __name__ == '__main__':
    runStowFSM()
