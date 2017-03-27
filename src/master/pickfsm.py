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

    def loadBoxFile(self, boxFile_location):
        with open(boxFile_location) as box_file:
            self.box_db = json.load(box_file)
        for i, n in self.box_db.items():
            for k in n:
                self.store.put('/box/box'+k['size_id'], k)

    def loadOrderFile(self, file_location):
        with open(file_location) as data_file:
            self.order_db = json.load(data_file)

        for i, n in self.order_db.items():
            for k in n:
                self.store.put('/order/order'+k['size_id'], k)
                self.store.put('/order/order'+k['size_id']+'/number', len(k['contents']))

        self.order = self.store.get('/order/').items()

        for i, n in self.order:
            for k in n['contents']:
                self.points = (20 if self.store.get('/item/'+k+'/new_item') else 10)
                self.points += 10/n['number']
                self.store.put('/item/'+k+'/point_value', self.points)
                self.store.put('/item/'+k+'/order', i)
                print "item ", k, " is valued at ", self.points, " for ", i

        #need to set all other items point values to 0 to ignore
        self.items = self.store.get('/item/').items()
        for i, n in self.items:
            if 'order' not in n.keys():
                self.store.put('/item/'+i+'/point_value', 0)


    def loadLocationFile(self, file_location):
        with open(file_location) as data_file:
            self.loc_db = json.load(data_file)
        #only get info. Putting info comes later

        for i, k in self.loc_db.items():
            if i == 'bins':
                for n in k:
                    self.store.put('/bins/bin'+n['bin_id'], n)
                    for l in n['contents']:
                        self.store.put('/item/'+l+'/location', 'bin'+n['bin_id'])

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
    #with open('data/test/workcell_032617.json.gz') as data_file:
    #    initial_db = json.load(data_file)
    #pick.store.put('', initial_db)
    pick.loadBoxFile('data/test/box_sizes.json')
    pick.store.delete('/order')
    pick.loadOrderFile('data/test/order_file.json')
    pick.loadLocationFile('data/test/item_location_file.json')
    pick.store.put('/robot/task', 'pick')
    #simulate for now
    pick.store.put('/simulate/robot_motion', True)
    pick.store.put('/simulate/object_detection', True)
    
    pick.setCurrentState('si')
   
    pick.runStep()
    while(not pick.isDone()): pick.runOrdered(pick.getCurrentState())

if __name__ == '__main__':
    runPickFSM()
