from master.fsm import State
from simulation.grabcut import GrabObject
import cv2
import numpy
import logging; logger = logging.getLogger(__name__)

class FindItem(State):
    def run(self):
        self.ItemDict = self.store.get('/item/'+self.store.get('/robot/selected_item'))
        #self.lastLoc = self.ItemDict['location']

        if self.store.get('/simulate/object_detection'):
            self.grab = GrabObject()
            self.grab.run() #output to db as mask
            cv2.destroyAllWindows()
            self.mask = self.store.get('/camera/camera1/mask')
            self.pc = self.store.get('/camera/camera1/point_cloud')
            self.binaryMask = (self.mask.sum(axis=2)>0)
            #cv2.imwrite('test/checkMask.png', self.binaryMask)
            self.obj_pc = self.pc[self.binaryMask]
            self.store.put('/item/'+self.ItemDict['name']+'/point_cloud', self.obj_pc)

            #fix camera tmp
            self.cam_pose = self.store.get('/camera/camera1/pose')
            #self.cam_pose = self.cam_pose.copy()
            #self.cam_pose[0,3]=self.cam_pose[0,3]+2
            #self.store.put('/camera/camera1/pose', self.cam_pose)

            #and need shelf pose
            self.shelf = self.store.get('/shelf/pose')

            #update pose as mean of obj pc
            self.tmp = self.obj_pc[:,2]>0
            self.pose = numpy.eye(4,4)
            self.pose[:3,3]=self.obj_pc[self.tmp].mean(axis=0)
            self.store.put('/item/'+self.ItemDict['name']+'/pose', numpy.linalg.inv(self.shelf).dot(self.pose))

        else:
            # for all cameras available, do:
            pass
        #etc
        #take camera pic
        #ID shelf, location, etc
        #get point cloud
 
        if self.foundLoc is not None:
            logger.info("Item found at {}".format(self.foundLoc))

        #self.foundLoc = self.lastLoc #maybe
        
        #self.store.put('/item/'+self.ItemDict['name']+'/location',  self.foundLoc)
        self.store.put('/status/selected_item_location', True)
        #etc

