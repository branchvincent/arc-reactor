import logging

import numpy

from master.fsm import State

from grasp import vacuum

from util.math_helpers import transform, rotate, crop_with_aabb
from util.location import location_bounds_url, location_pose_url

logger = logging.getLogger(__name__)

class EvaluateGrasp(State):
    '''
    Inputs:  /robot/target_location (e.g., 'binA')

    Outputs: photo_url + '/vacuum_grasps' (a list of lists of (grasp xform in WCS, grasp feature vectors))
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
        self.store.put('/robot/target_photo_url', photo_url)

        # retrieve the photo
        point_cloud = self.store.get(photo_url + ['point_cloud_segmented'])
        camera_pose = self.store.get(photo_url + ['pose'])
        labeled_image = self.store.get(photo_url + ['labeled_image'])
        full_color = self.store.get(photo_url + ['full_color'])

        # retreive the container and its bounds
        bounds_pose = self.store.get(location_pose_url(location))
        bounding_box = self.store.get(location_bounds_url(location))

        # generate world and local to container versions
        world_point_cloud = transform(camera_pose, point_cloud)
        local_point_cloud = transform(numpy.linalg.inv(bounds_pose), world_point_cloud)

        # apply bounding box
        mask = crop_with_aabb(local_point_cloud, bounding_box, point_cloud[:,:,2] > 0)

        # build the object point clouds
        object_masks = [numpy.logical_and(labeled_image == (idx + 1), mask) for idx in range(labeled_image.max())]
        logger.info('generated {} object masks'.format(len(object_masks)))

        # do not mask the full cloud because it must be structured
        grasps = vacuum.compute(local_point_cloud, object_masks, aligned_color=full_color)
        #TODO create pass/fail criteria

        for grasp in grasps:
            grasp['segment_id'] = self.find_segment_by_point(local_point_cloud, labeled_image, grasp['center'])
            grasp['center'] = transform(bounds_pose, grasp['center'])
            grasp['orientation'] = rotate(bounds_pose, grasp['orientation'][:3])

        logger.info('found {} grasps'.format(len(grasps)))
        logger.debug('{}'.format(grasps))

        # store result
        self.store.put(photo_url + ['vacuum_grasps'], grasps)

        self.setOutcome(True)
        logger.info('evaluate vacuum grasp completed successfully')

    def find_segment_by_point(self, point_cloud, labeled_image, grasp_center):
        # find the closest point in the point cloud
        distance = ((point_cloud - grasp_center)**2).sum(axis=2)
        distance[labeled_image == 0] = float('Inf')
        idx = numpy.unravel_index(numpy.argmin(distance), distance.shape)

        # lookup the matching segment index
        label = int(labeled_image[idx])
        if label == 0:
            raise RuntimeError('grasp center does not belong to segment: {}'.format(grasp_center))

        return label

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'eg')
    EvaluateGrasp(myname).run()
