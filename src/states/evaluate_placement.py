import logging

import numpy
from math import pi

from time import time

from master.fsm import State
from master.world import xyz, rpy

from packing import heightmap

from util.math_helpers import transform, crop_with_aabb
from util.location import location_bounds_url, location_pose_url

logger = logging.getLogger(__name__)

class EvaluatePlacement(State):
    '''
    Inputs:  /robot/target_locations (e.g., ['binA'])

    Outputs: /robot/placements: a dictionary of location names to 4x4 numpy matrix of end-effector pose
    '''

    def run(self):
        logger.info('finding placements')

        # obtain item point cloud from inspection station in tool coordinates
        item_cloud = self._build_item_point_cloud()

        placements = {}

        # attempt packing for each location
        for location in self.store.get('/robot/target_locations'):
            # obtain container point cloud in container local coordinates
            container_cloud = self._build_container_point_cloud(location)

            # attempt the packing
            container_aabb = self.store.get(location_bounds_url(location))
            (position, orientation) = heightmap.pack([container_cloud], item_cloud, [container_aabb])
            logger.info('found placement in "{}"'.format(location))
            logger.debug('{}, {}'.format(position, orientation))

            # transform placement into world coordinate system
            local_placement = xyz(*position) * rpy(0, 0, orientation * 180.0 / pi)
            container_pose = self.store.get(location_pose_url(location))
            world_placement = transform(container_pose, local_placement)

            placements[location] = world_placement

        # store result
        self.store.put('/robot/placements', placements)

        self.setOutcome(True)
        logger.info('find placement completed successfully')

    def _build_item_point_cloud(self):
        inspect_cloud_world = numpy.array([]).reshape((0, 3))
        inspect_cloud_color = numpy.array([]).reshape((0, 3))

        for photo_url in ['/photos/inspect/inspect_side', '/photos/inspect/inspect_below']:
            camera_pose = self.store.get(photo_url + '/pose')
            cloud_camera = self.store.get(photo_url + '/point_cloud')
            aligned_color = self.store.get(photo_url + '/aligned_color')
            cloud_world = transform(camera_pose, cloud_camera)
            valid_mask = cloud_camera[..., 2] > 0

            inspect_cloud_world = numpy.vstack((
                inspect_cloud_world,
                cloud_world[valid_mask]
            ))
            inspect_cloud_color = numpy.vstack((
                inspect_cloud_color,
                aligned_color[valid_mask]
            ))

        # move the point cloud into inspection local coordinates in preparation for cropping
        inspect_pose = self.store.get('/robot/inspect_pose')
        inspect_cloud_local = transform(numpy.linalg.inv(inspect_pose), inspect_cloud_world)
        inspect_bounds = self.store.get('/robot/inspect_bounds')

        # apply the inspection station bounding box
        crop_mask = crop_with_aabb(inspect_cloud_local, inspect_bounds)
        item_cloud = inspect_cloud_local[crop_mask]
        item_color = inspect_cloud_color[crop_mask]

        # save the point cloud for debugging
        self.store.multi_put({
            'point_cloud': item_cloud,
            'point_cloud_color': item_color,
            'timestamp': time()
        }, root='/debug/item')

        return item_cloud

    def _build_container_point_cloud(self, location):
        available_cameras = self.store.get(['system', 'viewpoints', location], [])

        # use end-effector camera for bins and fixed cameras otherwise
        if 'tcp' in available_cameras:
            if 'bin' in location:
                available_cameras = ['tcp']
            else:
                available_cameras.remove('tcp')

        if not available_cameras:
            raise RuntimeError('no camera available for {}'.format(location))
        logger.debug('available cameras for "{}": {}'.format(location, available_cameras))

        # TODO: choose camera a better way
        camera = available_cameras[0]
        photo_url = ['photos', location, camera]
        logger.info('using photo "{}" for location "{}"'.format(photo_url, location))

        photo_pose = self.store.get(photo_url + ['pose'])
        container_pose = self.store.get(location_pose_url(location))
        container_aabb = self.store.get(location_bounds_url(location))

        photo_aligned_color = self.store.get(photo_url + ['aligned_color'])
        photo_cloud_camera = self.store.get(photo_url + ['point_cloud'])
        photo_cloud_world = transform(photo_pose, photo_cloud_camera)
        photo_cloud_container = transform(numpy.linalg.inv(container_pose), photo_cloud_world)

        photo_valid_mask = (photo_cloud_camera[..., 2] > 0)
        container_cloud = photo_cloud_container[photo_valid_mask]
        container_color = photo_aligned_color[photo_valid_mask]

        # save the point cloud for debugging
        self.store.multi_put({
            'point_cloud': container_cloud,
            'point_cloud_color': container_color,
            'timestamp': time()
        }, root='/debug/container/{}'.format(location))

        return container_cloud

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ep')
    EvaluatePlacement(myname).run()
