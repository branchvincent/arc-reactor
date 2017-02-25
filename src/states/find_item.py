from master.fsm import State
from simulation.grabcut import GrabObject

from hardware.SR300 import DepthCameras

import cv2
import numpy
import logging; logger = logging.getLogger(__name__)

class FindItem(State):
    def run(self):
        self.ItemDict = self.store.get('/item/'+self.store.get('/robot/selected_item'))
        self.lastLoc = self.ItemDict['location']

        if self.store.get('/simulate/object_detection'):
            logger.info('simulating object detection')

            # acquire a camera image
            cam = DepthCameras()
            if not cam.connect():
                raise RuntimeError('failed accessing camera')

            (images, serial) = cam.acquire_image(0)
            if not serial:
                raise RuntimeError('camera acquisition failed')
            
            (color, aligned_color, _, _, _, self.pc) = images

            logger.debug('acquired image from camera {}'.format(serial))
            self.store.put('/camera/camera1/color_image', color)
            self.store.put('/camera/camera1/aligned_color_image', self.pc)

            # perform grab cut selection on color-aligned-to-depth image
            self.grab = GrabObject(images[1])
            self.mask = self.grab.run() 

            cv2.destroyAllWindows()

            self.binaryMask = (self.mask.sum(axis=2) > 0)
            #cv2.imwrite('test/checkMask.png', self.binaryMask)
            self.obj_pc = self.pc[numpy.bitwise_and(self.binaryMask, self.pc[:, :, 2] > 0)]
            self.store.put('/item/'+self.ItemDict['name']+'/point_cloud', self.obj_pc)
            logger.debug('found {} object points'.format(self.obj_pc.shape[0]))

            #fix camera tmp
            self.cam_pose = self.store.get('/camera/camera1/pose')
            #self.cam_pose = self.cam_pose.copy()
            #self.cam_pose[0,3]=self.cam_pose[0,3]+2
            #self.store.put('/camera/camera1/pose', self.cam_pose)

            #and need shelf pose
            self.shelf = self.store.get('/shelf/pose')

            #update pose as mean of obj pc
            self.pose = numpy.eye(4, 4)
            self.pose[:3, 3] = self.obj_pc.mean(axis=0)
            logger.debug('object pose relative to camera\n{}'.format(self.pose))

            obj_pose = numpy.linalg.inv(self.shelf).dot(self.pose)
            logger.debug('object pose relative to shelf\n{}'.format(obj_pose))

            self.store.put(['item', self.ItemDict['name'], 'pose'], obj_pose)

        else:
            # for all cameras available, do:
            pass
        #etc
        #take camera pic
        #ID shelf, location, etc
        #get point cloud

        self.foundLoc = self.lastLoc #maybe
 
        if self.foundLoc is not None:
            logger.info("Item found at {}".format(self.foundLoc))

        
        #self.store.put('/item/'+self.ItemDict['name']+'/location',  self.foundLoc)
        self.store.put('/status/selected_item_location', True)
        #etc

if __name__ == '__main__':
    FindItem('fi').run()
