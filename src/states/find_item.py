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

        # choose camera
        if location == 'binA':
            camera = 'shelf1'
        elif location == 'binB':
            camera = 'shelf0' #TODO: add overlapping camera 'shelf1'
        elif location == 'binC':
            camera = 'shelf0'
        elif location in ['stow_tote', 'stow tote']:
            camera = 'stow'
        else:
            raise RuntimeError('no camera available for {}'.format(location))

        if self.store.get('/simulate/object_detection'):
            logger.warn('simulating object detection of "{}"'.format(selected_item))

            if self.store.get('/simulate/cameras'):
                logger.warn('simulating cameras')

                # load previously acquired images in BGR format
                if camera == 'shelf0':
                    color = cv2.imread('data/simulation/color-shelf-0.png')[:, :, ::-1]
                    aligned_color = cv2.imread('data/simulation/aligned-shelf-0.png')[:, :, ::-1]
                    point_cloud = numpy.load('data/simulation/pc-shelf-0.npy')
                elif camera == 'shelf1':
                    color = cv2.imread('data/simulation/color-shelf-1.png')[:, :, ::-1]
                    aligned_color = cv2.imread('data/simulation/aligned-shelf-1.png')[:, :, ::-1]
                    point_cloud = numpy.load('data/simulation/pc-shelf-1.npy')
                elif camera == 'stow':
                    color = cv2.imread('data/simulation/color-stow_tote-0.png')[:, :, ::-1]
                    aligned_color = cv2.imread('data/simulation/aligned-stow_tote-0.png')[:, :, ::-1]
                    point_cloud = numpy.load('data/simulation/pc-stow_tote-0.npy')
                else:
                    raise RuntimeError('no simulated camera image available for {}'.format(location))

            else:
                from hardware.SR300 import DepthCameras

                # connect cameras
                cams = DepthCameras()
                if not cams.connect():
                    raise RuntimeError('failed accessing cameras')

                # find desired camera's serial number
                db_cams = self.store.get('/system/cameras')
                try:
                    serial_num = db_cams[camera]
                except:
                    raise RuntimeError('could not find serial number for camera "{}" in database'.format(camera))

                # acquire desired camera by serial number
                desired_cam_index = cams.get_camera_index_by_serial(serial_num)
                if desired_cam_index is None:
                    raise RuntimeError('could not find camera with serial number {}'.format(serial_num))

                # acquire a camera image
                (images, serial) = cams.acquire_image(desired_cam_index)
                if not serial:
                    raise RuntimeError('camera acquisition failed')

                (color, aligned_color, _, _, _, point_cloud) = images

                logger.debug('acquired image from camera {}'.format(serial))

            self.store.put(['camera', camera, 'color_image'], color)
            self.store.put(['camera', camera, 'aligned_image'], aligned_color)
            self.store.put(['camera', camera, 'point_cloud'], point_cloud)
            self.store.put(['camera', camera, 'timestamp'], time())

            if self.store.get('/test/skip_grabcut', False):
                mask = aligned_color
                logger.warn('skipped grabcut for testing')
            else:
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

            if location.startswith('bin'):
                reference_pose = self.store.get('/shelf/pose')
            elif location in ['stow_tote', 'stow tote']:
                reference_pose = self.store.get('/tote/stow/pose')
            else:
                raise RuntimeError('unrecognized item location: {}'.format(selected_item))

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
