import logging

import numpy

from master.fsm import State
from master.world import xyz

from packing import heightmap

from util.math_helpers import transform
from util.location import location_bounds_url, location_pose_url

logger = logging.getLogger(__name__)

class FindPlacement(State):
    '''
    Inputs:  /robot/target_location (e.g., 'binA')
             /robot/inspect_point_cloud releative to inspect_pose

    Outputs: /robot/target_placement (4x4 numpy matrix of end-effector pose)
    '''

    def run(self):
        logger.info('finding placements')

        # figure out which cameras to use
        location = self.store.get('/robot/target_location')
        available_cameras = self.store.get(['system', 'viewpoints', location], [])

        # use end-effector camera for bins and fixed cameras otherwise
        if 'tcp' in available_cameras:
            if 'bin' in location:
                available_cameras = ['tcp']
            else:
                available_cameras.remove('tcp')

        if not available_cameras:
            raise RuntimeError('no camera available for {}'.format(location))

        # TODO: choose camera a better way
        camera = available_cameras[0]
        photo_url = ['photos', location, camera]
        logger.info('using photo: {}'.format(photo_url))

        # retrieve the photo
        point_cloud = self.store.get(photo_url + ['point_cloud'])
        camera_pose = self.store.get(photo_url + ['pose'])

        # mask out invalid points
        mask = (point_cloud[:,:,2] > 0)

        # retreive the container and its bounds
        bounds_pose = self.store.get(location_pose_url(location))
        bounding_box = self.store.get(location_bounds_url(location))

        # transform the point cloud into the local frame
        world_point_cloud = transform(camera_pose, point_cloud)
        local_point_cloud = transform(numpy.linalg.inv(bounds_pose), world_point_cloud)

        # perform packing
        item_point_cloud = self.store.get('/robot/inspect_point_cloud')
        # TODO: transform item point cloud in local coordinate system

        # HACK: just use the bounds for now
        logger.error('assuming the object is box (not using inspect point cloud')
        # TODO: handle orientation
        position = heightmap.pack([local_point_cloud[mask]], [0.1, 0.1, 0.1] , [bounding_box])
        logger.info('found placement')
        logger.debug('{}'.format(position))

        # transform placement into world coordinate system
        local_placement = xyz(*position)
        world_placement = bounds_pose.dot(local_placement)

        # store result
        self.store.put('/robot/target_placement', world_placement)

        self.setOutcome(True)
        logger.info('find placement completed successfully')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'fp')
    FindPlacement(myname).run()
