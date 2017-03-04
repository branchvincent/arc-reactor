from master.fsm import State

import cv2
import numpy
import logging; logger = logging.getLogger(__name__)

class FindItem(State):
    def run(self):
        selected_item = self.store.get('/robot/selected_item')
        self.ItemDict = self.store.get(['item', selected_item])

        if self.store.get('/simulate/object_detection'):
            logger.warn('simulating object detection of "{}"'.format(selected_item))

            if self.store.get('/simulate/cameras'):
                logger.warn('simulating cameras')

                # load previously acquired images in BGR format
                color = cv2.imread('data/simulation/color-shelf-0.png')[:, :, ::-1]
                aligned_color = cv2.imread('data/simulation/aligned-shelf-0.png')[:, :, ::-1]
                point_cloud = numpy.load('data/simulation/pc-shelf-0.npy')
            else:
                from hardware.SR300 import DepthCameras

                # acquire a camera image
                cam = DepthCameras()
                if not cam.connect():
                    raise RuntimeError('failed accessing camera')

                (images, serial) = cam.acquire_image(0)
                if not serial:
                    raise RuntimeError('camera acquisition failed')

                (color, aligned_color, _, _, _, point_cloud) = images

                logger.debug('acquired image from camera {}'.format(serial))

            self.store.put('/camera/camera1/color_image', color)
            self.store.put('/camera/camera1/aligned_image', aligned_color)
            self.store.put('/camera/camera1/point_cloud', point_cloud)

            # perform grab cut selection on color-aligned-to-depth image
            from simulation.grabcut import GrabObject
            self.grab = GrabObject(aligned_color)
            self.mask = self.grab.run()

            cv2.destroyAllWindows()

            self.binaryMask = (self.mask.sum(axis=2) > 0)
            #cv2.imwrite('test/checkMask.png', self.binaryMask)
            self.obj_pc = point_cloud[numpy.bitwise_and(self.binaryMask, point_cloud[:, :, 2] > 0)]
            mean = self.obj_pc.mean(axis=0)
            self.store.put('/item/'+self.ItemDict['name']+'/point_cloud', self.obj_pc - mean)
            logger.debug('found {} object points'.format(self.obj_pc.shape[0]))

            #fix camera tmp
            self.cam_pose = self.store.get('/camera/camera1/pose')

            #and need shelf pose
            self.shelf = self.store.get('/shelf/pose')

            #update pose as mean of obj pc
            self.pose = numpy.eye(4, 4)
            self.pose[:3, 3] = mean
            logger.debug('object pose relative to camera\n{}'.format(self.pose))

            obj_pose = numpy.linalg.inv(self.shelf).dot(self.cam_pose.dot(self.pose))
            logger.debug('object pose relative to shelf\n{}'.format(obj_pose))

            self.store.put(['item', self.ItemDict['name'], 'pose'], obj_pose)

        else:
            # for all cameras available, do:
            pass
        #etc
        #take camera pic
        #ID shelf, location, etc
        #get point cloud

        
        self.store.put('/status/selected_item_location', True)
        #etc

if __name__ == '__main__':
    FindItem('fi').run()
