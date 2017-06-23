import logging

import numpy

from master.fsm import State
from master.world import rpy

from grasp import pinch

from util.math_helpers import transform, rotate, crop_with_aabb
from util.location import location_bounds_url, location_pose_url

from . import NoViewingCameraError, MissingPhotoError, MissingSegmentationError, GraspNotInSegmentError

logger = logging.getLogger(__name__)

class EvaluatePinchGraspBase(State):
    def _common(self, locations):
        logger.info('starting pinch grasp evaluation for {}'.format(locations))

        try:
            for location in locations:
                self._handle(location)
        except (NoViewingCameraError, MissingPhotoError, MissingSegmentationError, GraspNotInSegmentError) as e:
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
            logger.exception('pinch grasp evaluation failed')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.setOutcome(True)

        logger.info('finished pinch grasp evaluation')

        if self.store.get('/debug/grasps', False):
            from util import db
            suffix = 'success' if self.getOutcome() else 'failure'
            db.dump(self.store, '/tmp/grasp-{}-{}'.format('-'.join(locations), suffix))
            logger.info('database dump completed')

    def _handle(self, location):
        logger.info('finding pinch grasps for {}'.format(location))

        # figure out which cameras to use
        available_cameras = self.store.get(['system', 'viewpoints', location], [])

        # use end-effector camera for bins and fixed cameras otherwise
        if 'tcp' in available_cameras:
            if 'bin' in location:
                available_cameras = ['tcp']
            else:
                available_cameras.remove('tcp')

        if not available_cameras:
            raise NoViewingCameraError('no camera available for location {}'.format(location))

        for camera in available_cameras:
            photo_url = ['photos', location, camera]
            logger.info('using photo: {}'.format(photo_url))
            self.store.put('/robot/target_photo_url', photo_url)

            # retrieve the photo
            try:
                camera_pose = self.store.get(photo_url + ['pose'], strict=True)
                point_cloud = self.store.get(photo_url + ['point_cloud_segmented'], strict=True)
                full_color = self.store.get(photo_url + ['full_color'], strict=True)
            except KeyError:
                raise MissingPhotoError('missing photo data for location {}: {}'.format(location, photo_url))

            labeled_image = self.store.get(photo_url + ['labeled_image'])
            if labeled_image is None:
                raise MissingSegmentationError('missing segmentation for location {}: {}'.format(location, photo_url))

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
            grasps = pinch.compute(local_point_cloud, masks=object_masks, image=full_color)
            #TODO create pass/fail criteria

            for (i, grasp) in enumerate(grasps):
                grasp['center'] = transform(bounds_pose, [grasp['center'][0], grasp['center'][1], grasp['tip_height']])
                grasp['orientation'] = bounds_pose.dot(rpy(0, 0, grasp['rotation']))
                grasp['segment_id'] = self._find_segment_by_point(local_point_cloud, labeled_image, grasp['center'])
                grasp['index'] = i
                grasp['location'] = location
                grasp['camera'] = camera

            # store result
            self.store.put(photo_url + ['pinch_grasps'], grasps)

            logger.info('found {} grasps for {} from {}'.format(len(grasps), location, camera))
            logger.debug('{}'.format(grasps))

        logger.info('finished finding pinch grasps')

    def _find_segment_by_point(self, point_cloud, labeled_image, grasp_center):
        # find the closest point in the point cloud
        distance = ((point_cloud - grasp_center)**2).sum(axis=2)
        distance[labeled_image == 0] = float('Inf')
        idx = numpy.unravel_index(numpy.argmin(distance), distance.shape)

        # lookup the matching segment index
        label = labeled_image[idx]
        if label == 0:
            raise GraspNotInSegmentError('grasp center does not belong to segment: {}'.format(grasp_center))

        return label
