import logging

import numpy

from master.fsm import State

from grasp import vacuum

from util.math_helpers import transform
from util.location import location_bounds_url, location_pose_url

logger = logging.getLogger(__name__)

class FindVacuumGrasp(State):
    '''
    Inputs:  /robot/target_location (e.g., 'binA')

    Outputs: /robot/vacuum_grasps (a list of lists of (grasp xform in WCS, grasp feature vectors))
    '''

    def run(self):
        logger.info('finding vacuum grasps')

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
        # HACK: actually need the labeled image registered with the point cloud
        logger.error('assuming that the labeled image is registered with the point cloud')
        object_mask = self.store.get(photo_url + ['labeled_image'])

        # retreive the container and its bounds
        bounds_pose = self.store.get(location_pose_url(location))
        bounding_box = self.store.get(location_bounds_url(location))

        # generate world and local to container versions
        world_point_cloud = transform(camera_pose, point_cloud)
        local_point_cloud = transform(numpy.linalg.inv(bounds_pose), world_point_cloud)

        # apply bounding box
        mask = (point_cloud[:,:,2] > 0)
        for dim in range(3):
            mask = numpy.logical_and(mask, local_point_cloud[:, :, dim] >= min([bounding_box[0][dim], bounding_box[1][dim]]))
            mask = numpy.logical_and(mask, local_point_cloud[:, :, dim] <= max([bounding_box[0][dim], bounding_box[1][dim]]))

        # build the object point clouds
        object_clouds = [ world_point_cloud[object_mask == (idx + 1)][mask] for idx in range(object_mask.max())]
        logger.info('generated {} object point clouds'.format(len(object_clouds)))

        # do not mask the full cloud because it must be structured
        grasps = vacuum.compute(world_point_cloud, object_clouds)
        logger.info('found {} grasps'.format(len(grasps)))
        logger.debug('{}'.format(grasps))

        # store result
        self.store.put('/robot/vacuum_grasps', grasps)

        self.setOutcome(True)
        logger.info('find vacuum grasp completed successfully')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'fvg')
    FindVacuumGrasp(myname).run()
