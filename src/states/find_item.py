from master.fsm import State
from simulation.grabcut import GrabObject
import cv2

class FindItem(State):
    def run(self):
        self.ItemDict = self.store.get('/item/'+self.store.get('/robot/selected_item'))
        self.lastLoc = self.ItemDict['location']

        if self.store.get('/simulate/object_detection'):
            self.grab = GrabObject()
            self.grab.run() #output to db as mask
            self.mask = self.store.get('/camera/camera1/mask')
            self.pc = self.store.get('/camera/camera1/point_cloud')
            self.binaryMask = (self.mask.sum(axis=2)>0)
            #cv2.imwrite('test/checkMask.png', self.binaryMask)
            self.obj_pc = self.pc[self.binaryMask]
            self.store.put('/item/'+self.ItemDict['name']+'/point_cloud', self.obj_pc)
        else:
            # for all cameras available, do:
            pass
        #etc
        #take camera pic
        #ID shelf, location, etc
        #get point cloud


        self.foundLoc = self.lastLoc #maybe
        
        self.store.put('/item/'+self.ItemDict['name']+'/location',  self.foundLoc)
        self.store.put('/status/selected_item_location', True)
        #etc

