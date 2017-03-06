from master.fsm import StateMachine
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute
from states.exec_route import ExecRoute
from states.check_item import CheckItem
from states.check_route import CheckRoute
from states.check_select_item import CheckSelectItem
from states.move_shelf import MoveShelf
import json
import cv2
import numpy

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

    def loadOrderFile(self, file_location):
        with open(file_location) as data_file:
            self.order_db = json.load(data_file)
        self.store.multi_put(self.order_db)
        self.order = self.store.get('/order/').items()
        #need to set all other items point values to 0 to ignore (?)
        for i, n in self.order:
            for k in n['items']:
                self.points = (20 if self.store.get('/item/'+k+'/new_item') else 10)
                self.points += 10/n['number']
                self.store.put('/item/'+k+'/point_value', self.points)
                self.store.put('/item/'+k+'/order', i)
                print "item ", k, " is valued at ", self.points, " for ", i

    def doneOrderFile(self):
        #if all items picked, all their point values are 0. Need to re-write
        self.value = 0
        for i, n in self.store.get('/order/').items():
            for k in n['items']:
                self.points = self.store.get('/item/'+k+'/point_value')
                self.value+=self.points
        return (self.value==0)
    isDone = doneOrderFile

#################################################################################
def runPickFSM():
    pick = PickStateMachine()
    pick.loadStates()
    pick.setupTransitions()
    with open('test/master/test_021317_2.json') as data_file:
        initial_db = json.load(data_file)
    pick.store.put('', initial_db)
    pick.loadOrderFile('test/master/order_test.json')
    pick.store.put('/robot/task', 'pick')
    #simulate for now
    pick.store.put('/simulate/robot_motion', True)
    pick.store.put('/simulate/object_detection', True)
    #image = cv2.imread('test/camera1.png')
    #pick.store.put('/camera/camera1/color_image', image)
    #pc = numpy.load('test/camera1_pc.npy')
    #pick.store.put('/camera/camera1/point_cloud', pc)
    #number = 10
    #for _ in range(number): pick.runOrdered('si')
    pick.setCurrentState('si')
    pick.runStep()
    pick.runStep()
    while(not pick.doneOrderFile()): pick.runOrdered(pick.getCurrentState())

if __name__ == '__main__':
    runPickFSM()
