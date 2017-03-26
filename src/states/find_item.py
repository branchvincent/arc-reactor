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

        # location camera map
        # /shelf/cameras/bins
        location_to_cameras = {
            'binA': ['self1']
            'binB': ['shelf0','shelf1']
            'binC': ['shelf0']
            'stow_tote': ['stow']
            'stow tote': ['stow']
        }

        # select cameras
        try:
            selected_cameras = location_to_cameras[location]
        else:
            raise RuntimeError('no camera available for {}'.format(location))

        # detect object
        if self.store.get('/simulate/object_detection', False):
            logger.warn('simulating object detection of "{}"'.format(selected_item))

            colors, aligned_colors, point_clouds = [],[],[]

            # get images
            if self.store.get('/simulate/cameras', False):
                logger.warn('simulating cameras')
                for camera in selected_cameras:
                    color, aligned_color, point_cloud = self.simulate_acquire_image(camera)
                    colors.append(color)
                    aligned_color.append(aligned_color)
                    point_clouds.append(point_cloud)
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
                    colors.append(color)
                    aligned_color.append(aligned_color)

                    # Point cloud in world coordinates
                    camera_pose = self.store.get(['camera', camera, 'pose'])
                    pc_world = point_cloud.dot(camera_pose[:3, :3].T) + camera_pose[:3, 3].T
                    point_clouds.append(camera_pose.dot(pc_world))

            # Perform grab cut
            cut_pc, cut_pc_color = [],[]
            for color,aligned_color,point_cloud in zip(colors,aligned_colors,point_clouds):
                mask = self.perform_grab_cut(aligned_color)
                cv2.destroyAllWindows()
                binaryMask = (mask.sum(axis=2) > 0)
                mask = numpy.bitwise_and(binaryMask, point_cloud[:, :, 2] > 0)
                obj_pc = point_cloud[mask]
                # Update global point cloud
                cut_pc = numpy.concatenate((cut_pc,obj_pc), axis=0)
                cut_pc_color = numpy.concatenate((cut_pc_color,aligned_colors), axis=0)

            # Update database
            mean = cut_pc.mean(axis=0)
            self.store.put(['item', selected_item, 'point_cloud'], cut_pc - mean)
            self.store.put(['item', selected_item, 'point_cloud_color'], cut_pc_color)
            self.store.put(['item', selected_item, 'timestamp'], time())

            logger.debug('found {} object points'.format(cut_pc.shape[0]))

            #update pose as mean of point cloud
            item_pose_world = numpy.eye(4)
            item_pose_world[:3, 3] = mean
            logger.debug('object pose relative to world\n{}'.format(item_pose_world))

            reference_pose = numpy.eye(4)
            if location.startswith('bin'):
                reference_pose = self.store.get('/shelf/pose')
            elif location in ['stow_tote', 'stow tote']:
                reference_pose = self.store.get('/tote/stow/pose')
            else:
                raise RuntimeError('unrecognized item location: {}'.format(selected_item))

            item_pose_reference = numpy.linalg.inv(reference_pose).dot((item_pose_world))
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

    def simulate_acquire_image(self, camera_name):
        """Simulates acquiring an image from the specified camera"""
        # Load simulated image data
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

    def perform_grab_cut(self, aligned_color):
        """Perform grab cut selection on color-aligned-to-depth image"""
        if self.store.get('/test/skip_grabcut', False):
            logger.warn('skipped grabcut for testing')
            mask = aligned_color
        else:
            from simulation.grabcut import GrabObject
            grab = GrabObject(aligned_color)
            mask = grab.run()
        return mask

if __name__ == '__main__':
    FindItem('fi').run()
