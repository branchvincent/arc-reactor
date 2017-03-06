from master.fsm import State

import cv2
import numpy

from time import time

import logging; logger = logging.getLogger(__name__)

class FindItem(State):
    def run(self):
        selected_item = self.store.get('/robot/selected_item')

        location = self.store.get(['item', selected_item, 'location'])

        logger.info('finding item "{}" in "{}"'.format(selected_item, location))

        if location == 'shelf':
            camera = 'shelf0'
        elif location in ['stow_tote', 'stow tote']:
            camera = 'stow'
        else:
            raise RuntimeError('no simulated camera image available for {}'.format(location))

        if self.store.get('/simulate/object_detection'):
            logger.warn('simulating object detection of "{}"'.format(selected_item))

            if self.store.get('/simulate/cameras'):
                logger.warn('simulating cameras')

                # load previously acquired images in BGR format
                if camera == 'shelf0':
                    color = cv2.imread('data/simulation/color-shelf-0.png')[:, :, ::-1]
                    aligned_color = cv2.imread('data/simulation/aligned-shelf-0.png')[:, :, ::-1]
                    point_cloud = numpy.load('data/simulation/pc-shelf-0.npy')
                elif camera == 'stow':
                    color = cv2.imread('data/simulation/color-stow_tote-0.png')[:, :, ::-1]
                    aligned_color = cv2.imread('data/simulation/aligned-stow_tote-0.png')[:, :, ::-1]
                    point_cloud = numpy.load('data/simulation/pc-stow_tote-0.npy')
                else:
                    raise RuntimeError('no simulated camera image available for {}'.format(location))

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

            self.store.put(['camera', camera, 'color_image'], color)
            self.store.put(['camera', camera, 'aligned_image'], aligned_color)
            self.store.put(['camera', camera, 'point_cloud'], point_cloud)
            self.store.put(['camera', camera, 'timestamp'], time())

            # perform grab cut selection on color-aligned-to-depth image
            from simulation.grabcut import GrabObject
            grab = GrabObject(aligned_color)
            mask = grab.run()

            cv2.destroyAllWindows()

            binaryMask = (mask.sum(axis=2) > 0)
            mask = numpy.bitwise_and(binaryMask, point_cloud[:, :, 2] > 0)
            obj_pc = point_cloud[mask]
            mean = obj_pc.mean(axis=0)

            self.store.put(['item', selected_item, 'point_cloud'], obj_pc - mean)
            self.store.put(['item', selected_item, 'point_cloud_color'], aligned_color[mask])
            self.store.put(['item', selected_item, 'timestamp'], time())

            logger.debug('found {} object points'.format(obj_pc.shape[0]))

            #update pose as mean of obj pc
            item_pose_camera = numpy.eye(4)
            item_pose_camera[:3, 3] = mean
            logger.debug('object pose relative to camera\n{}'.format(item_pose_camera))

            camera_pose = self.store.get(['camera', camera, 'pose'])
            reference_pose = numpy.eye(4)

            if location == 'shelf':
                #and need shelf pose
                reference_pose = self.store.get('/shelf/pose')
            elif location in ['stow_tote', 'stow tote']:
                reference_pose = self.store.get('/tote/stow/pose')

            item_pose_reference = numpy.linalg.inv(reference_pose).dot(camera_pose.dot(item_pose_camera))
            logger.debug('object pose relative to {}\n{}'.format(location, item_pose_reference))

            self.store.put(['item', selected_item, 'pose'], item_pose_reference)

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
