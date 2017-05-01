from master.fsm import State

import cv2
import numpy

from time import time

from subprocess import check_call

from util.math import transform

import logging; logger = logging.getLogger(__name__)

class FindItem(State):
    def run(self):
        selected_item = self.store.get('/robot/selected_item')
        location = self.store.get(['item', selected_item, 'location'])
        logger.info('finding item "{}" in "{}"'.format(selected_item, location))

        # select cameras
        selected_cameras = self.store.get(['system', 'viewpoints', location], None)
        if selected_cameras is None:
            raise RuntimeError('no camera available for {}'.format(location))

        # detect object
        if self.store.get('/simulate/object_detection', False):
            logger.warn('simulating object detection of "{}"'.format(selected_item))
            pc, pc_color = numpy.array([[]]),numpy.array([[]])

            # get images
            if self.store.get('/simulate/cameras', False):
                logger.warn('simulating cameras')
                for camera in selected_cameras:
                    color, aligned_color, point_cloud = self.simulate_acquire_image(camera)

                    # Perform grab cut
                    obj_pc_color, obj_pc = self.perform_grab_cut(aligned_color, point_cloud)

                    # Convert to world coorindates and update global point cloud
                    camera_pose = self.store.get(['camera', camera, 'pose'])
                    obj_pc_world = transform(camera_pose, obj_pc)
                    if pc.size == 0:
                        pc = obj_pc_world
                        pc_color = obj_pc_color
                    else:
                        pc = numpy.concatenate((pc, obj_pc_world), axis=0)
                        pc_color = numpy.concatenate((pc_color, obj_pc_color), axis=0)
            else:
                from hardware.SR300 import DepthCameras

                # connect cameras
                cameras = DepthCameras()
                if not cameras.connect():
                    raise RuntimeError('failed accessing cameras')

                # acquire camera images
                camera_serials = self.store.get('/system/cameras')
                for camera in selected_cameras:
                    color, aligned_color, point_cloud = self.acquire_image(cameras, camera, camera_serials)

                    # Perform grab cut
                    obj_pc_color, obj_pc = self.perform_grab_cut(aligned_color, point_cloud)

                    # Convert to world coorindates and update global point cloud
                    camera_pose = self.store.get(['camera', camera, 'pose'])
                    obj_pc_world = transform(camera_pose, obj_pc)
                    if pc.size == 0:
                        pc = obj_pc_world
                        pc_color = obj_pc_color
                    else:
                        pc = numpy.concatenate((pc,obj_pc_world), axis=0)
                        pc_color = numpy.concatenate((pc_color,obj_pc_color), axis=0)
        else:
            BASE_PATH = '/home/motion/Desktop/reactor-perception/src/perception/'

            # XXX: right now we can only properly handle a single camera
            selected_cameras = selected_cameras[:1]
            name2serial = self.store.get('/system/cameras')
            serials  = [name2serial[n] for n in selected_cameras]

            # get images
            if self.store.get('/simulate/cameras', False):
                for camera in selected_cameras:
                    self.simulate_acquire_image(camera)
            else:
                # XXX: the segmenter acquires images anyways so skip it here
                # check_call(['python3', BASE_PATH + 'perception.py', 'acquire'] + serials, cwd=BASE_PATH)
                pass

            # run the segmenter
            check_call(['python3', BASE_PATH + 'perception.py', 'segment'] + serials + [location], cwd=BASE_PATH)

            # retrieve the resultant point cloud which is in camera coordinates
            obj_pc_local = self.store.get(['item', selected_item, 'point_cloud'])
            # XXX: right now we can only properly handle a single camera
            camera_pose = self.store.get(['camera', selected_cameras[0], 'pose'])
            pc = transform(camera_pose, obj_pc_local)
            # XXX: no color image yet either
            pc_color = None

        # Update pose as mean of point cloud
        mean = pc.mean(axis=0)
        item_pose_world = numpy.eye(4)
        item_pose_world[:3,3] = mean
        logger.debug('object pose relative to world\n{}'.format(item_pose_world))

        reference_pose = numpy.eye(4)
        if location.startswith('bin'):
            reference_pose = self.store.get('/shelf/pose')
        elif location in ['stow_tote', 'stow tote']:
            reference_pose = self.store.get('/tote/stow/pose')
        else:
            raise RuntimeError('unrecognized item location: {}'.format(selected_item))

        item_pose_reference = numpy.linalg.inv(reference_pose).dot(item_pose_world)
        logger.debug('object pose relative to {}\n{}'.format(location, item_pose_reference))
        self.store.put(['item', selected_item, 'pose'], item_pose_reference)

        # Update item point cloud in local coordinates
        inv_pose_world = numpy.linalg.inv(item_pose_world)
        pc_local = transform(inv_pose_world, pc)
        mean_local = pc_local.mean(axis=0)
        self.store.put(['item', selected_item, 'point_cloud'], pc_local - mean_local)
        self.store.put(['item', selected_item, 'point_cloud_color'], pc_color)
        self.store.put(['item', selected_item, 'timestamp'], time())

        logger.debug('found {} object points'.format(pc.shape[0]))

        self.store.put('/status/selected_item_location', True)

    def simulate_acquire_image(self, camera_name):
        """Simulates acquiring an image from the specified camera"""
        # Load simulated image data
        print "Camera name",camera_name
        color = cv2.imread('data/simulation/color-{}-0.png'.format(camera_name))[:, :, ::-1]
        aligned_color = cv2.imread('data/simulation/aligned-{}-0.png'.format(camera_name))[:, :, ::-1]
        point_cloud = numpy.load('data/simulation/pc-{}-0.npy'.format(camera_name))

        # Update database
        self.store.put(['camera', camera_name, 'color_image'], color)
        self.store.put(['camera', camera_name, 'aligned_image'], aligned_color)
        self.store.put(['camera', camera_name, 'point_cloud'], point_cloud)
        self.store.put(['camera', camera_name, 'timestamp'], time())

        return color, aligned_color, point_cloud

    def acquire_image(self, cameras, camera_name, camera_serials):
        """Acquires an image from the specified camera"""
        # find desired camera's serial number
        try:
            serial_num = camera_serials[camera_name]
        except:
            raise RuntimeError('could not find serial number for camera "{}" in database'.format(camera_name))

        # acquire desired camera by serial number
        desired_cam_index = cameras.get_camera_index_by_serial(serial_num)
        if desired_cam_index is None:
            raise RuntimeError('could not find camera with serial number {}'.format(serial_num))

        # acquire a camera image
        (images, serial) = cameras.acquire_image(desired_cam_index)
        if not serial:
            raise RuntimeError('camera acquisition failed')

        (color, aligned_color, _, _, _, point_cloud) = images

        logger.debug('acquired image from camera {}'.format(serial))

        # Update database
        self.store.put(['camera', camera_name, 'color_image'], color)
        self.store.put(['camera', camera_name, 'aligned_image'], aligned_color)
        self.store.put(['camera', camera_name, 'point_cloud'], point_cloud)
        self.store.put(['camera', camera_name, 'timestamp'], time())

        return color, aligned_color, point_cloud

    def perform_grab_cut(self, aligned_color, point_cloud):
        """Perform grab cut selection on color-aligned-to-depth image"""
        # Get mask
        if self.store.get('/test/skip_grabcut', False):
            logger.warn('skipped grabcut for testing')
            mask = aligned_color
        else:
            from simulation.grabcut import GrabObject
            grab = GrabObject(aligned_color)
            mask = grab.run()
        # Apply mask
        cv2.destroyAllWindows()
        binaryMask = (mask.sum(axis=2) > 0)
        mask = numpy.bitwise_and(binaryMask, point_cloud[:, :, 2] > 0)
        return aligned_color[mask], point_cloud[mask]

if __name__ == '__main__':
    FindItem('fi').run()
