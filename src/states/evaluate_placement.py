import logging

from time import time

from math import pi
import numpy

from master.fsm import State
from master.world import xyz, rpy

from packing import heightmap

from util.math_helpers import transform, crop_with_aabb, zero_translation
from util.location import location_bounds_url, location_pose_url
from util.filtering import distance_label
from util import photo

from .common import NoViewingCameraError, MissingPhotoError

logger = logging.getLogger(__name__)

class NoPlacementFound(Exception):
    pass

class EvaluatePlacement(State):
    '''
    Inputs:
     - /robot/target_locations (e.g., ['binA'])
     - photos for each target location

    Outputs:
     - /robot/placement/pose: a 4x4 numpy matrix of end-effector pose
     - /robot/placement/location: location name of placement
     - /robot/target_bin and /robot/target_location: set to location of placement

    Failures:
     - NoViewingCameraError: no camera viewing packing locations
     - MissingPhotoError: missing photo from inspection station only
     - NoPlacementFound: cannot find placement

    Dependencies:
     - CapturePhoto for each location
    '''

    def run(self):
        locations = self.store.get('/robot/target_locations', [])
        logger.info('finding placement in {}'.format(locations))

        try:
            self._handler(locations)
        except (NoViewingCameraError, MissingPhotoError, NoPlacementFound) as e:
            self.store.put(['failure', self.getFullName()], e.__class__.__name__)
            logger.exception('placement finding failed')
        else:
            self.store.delete(['failure', self.getFullName()])
            self.store.put('/status/ep_done', False)
            self.setOutcome(True)

        logger.info('finished placement evaluation')

        if self.store.get('/debug/placements', False):
            from util import db
            suffix = 'success' if self.getOutcome() else 'failure'

            logger.info('database dump started')
            db.dump(self.store, '/tmp/placement-{}-{}'.format('-'.join(locations), suffix))
            logger.info('database dump completed')

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None or self.whyFail=="NoPlacementFound"):
            check = self.store.get('/status/ep_done', False)
            if(check):
                return 2
            else:
                self.store.put('/status/ep_done', True)
                return 0
        elif(self.whyFail == "MissingPhotoError"):
            return 1
        else:
            return 2

    def _handler(self, locations):
        # obtain item point cloud from inspection station in tool coordinates
        item_cloud = self._build_item_point_cloud()

        # obtain container point cloud and bounding box in container local coordinates
        container_clouds = [self._build_container_point_cloud(location) for location in locations]
        container_corners = [self._build_container_corners(location) for location in locations]

        robot_pose_world = self.store.get('/robot/inspect_pose')

        if locations == ['amnesty_tote']:
            logger.warn('using hardcoded amnesty drop site')

            idx = 0
            position = self.store.get('/robot/amnesty_drop_position')
            orientation = 0

        else:

            # attempt the packing
            margin = self.store.get('/packing/margin', 0.03)
            (idx, position, orientation, _) = heightmap.pack(
                container_clouds,
                item_cloud,
                container_corners,
                list(robot_pose_world[:3, 3].flat),
                margin=margin
            )

        if idx is not None:
            # packing succeeded
            pack_location = locations[idx]
            logger.info('found placement in "{}"'.format(pack_location))
            logger.debug('position {}, rotation {}'.format(position, orientation))

            # increase the placement by the offset
            selected_item = self.store.get('/robot/selected_item')
            placement_offset = self.store.get(['item', selected_item, 'placement_offset'], 0.01)
            if placement_offset is not None:
                position[2] += placement_offset
                logger.warn('using placement offset for "{}": {}'.format(selected_item, placement_offset))

            # transform placement into world coordinate system
            robot_pose_world = self.store.get('/robot/tcp_pose')

            world_placement = xyz(*position).dot(rpy(0, 0, orientation * pi / 180.0)).dot(zero_translation(robot_pose_world))

            # store result
            self.store.put('/robot/placement', {'pose': world_placement, 'location': pack_location})

            self.store.put('/robot/target_bin', pack_location)
            self.store.put('/robot/target_location', pack_location)

            self.setOutcome(True)
            logger.info('find placement succeeded')
        else:
            # packing failed
            self.store.delete('/robot/target_bin')
            self.store.delete('/robot/target_location')
            self.store.delete('/robot/placement')

            raise NoPlacementFound(locations)

    def _build_item_point_cloud(self):
        inspect_cloud_world = numpy.array([]).reshape((0, 3))
        inspect_cloud_color = numpy.array([]).reshape((0, 3))

        for photo_url in ['/photos/inspect/inspect_side', '/photos/inspect/inspect_below']:
            try:
                camera_pose = self.store.get(photo_url + '/pose', strict=True)
                cloud_camera = self.store.get(photo_url + '/point_cloud', strict=True)
                aligned_color = self.store.get(photo_url + '/aligned_color', strict=True)
            except KeyError:
                self.store.put(['failure', '{}_message'.format(self.getFullName())], photo.url2location_camera(photo_url)[0])
                raise MissingPhotoError('missing photo data for inspection: {}'.format(photo_url))

            cloud_world = transform(camera_pose, cloud_camera)
            #valid_mask = self._filter_cloud(cloud_world, cloud_camera[..., 2] > 0)
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
        item_cloud_local = inspect_cloud_local[crop_mask]
        item_color = inspect_cloud_color[crop_mask]

        item_cloud_world = transform(inspect_pose, item_cloud_local)

        # save the point cloud for debugging
        self.store.multi_put({
            'point_cloud': item_cloud_world,
            'point_cloud_color': item_color,
            'timestamp': time()
        }, root='/debug/item')

        return item_cloud_world

    def _build_container_corners(self, location):
        container_aabb = self.store.get(location_bounds_url(location))

        # generate four corners from bounding box
        local_points = container_aabb[:]
        local_points.append(local_points[0][0:1] + local_points[1][1:3])
        local_points.append(local_points[1][0:1] + local_points[0][1:3])

        # transform into world space
        container_pose = self.store.get(location_pose_url(location))
        world_points = [list(transform(container_pose, point).flat) for point in local_points]

        logger.debug('generated corners for "{}":\n{}'.format(location, numpy.array(world_points)))

        return world_points

    def _build_container_point_cloud(self, location):
        available_cameras = self.store.get(['system', 'viewpoints', location], [])

        # use end-effector camera for bins and fixed cameras otherwise
        if 'tcp' in available_cameras:
            if 'bin' in location:
                available_cameras = ['tcp']
            else:
                available_cameras.remove('tcp')

        if not available_cameras:
            raise NoViewingCameraError('no camera available for {}'.format(location))
        logger.debug('available cameras for "{}": {}'.format(location, available_cameras))

        container_pose = self.store.get(location_pose_url(location))
        container_aabb = self.store.get(location_bounds_url(location))

        container_clouds = []
        container_colors = []

        # process the point cloud from each viewpoint
        for camera in available_cameras:
            photo_url = ['photos', location, camera]
            logger.info('using photo "{}" for location "{}"'.format(photo_url, location))

            photo_pose = self.store.get(photo_url + ['pose'])
            if photo_pose is None:
                return numpy.array([]).reshape((0, 3))

            photo_aligned_color = self.store.get(photo_url + ['aligned_color'])
            photo_cloud_camera = self.store.get(photo_url + ['point_cloud'])
            photo_cloud_world = transform(photo_pose, photo_cloud_camera)
            photo_cloud_container = transform(numpy.linalg.inv(container_pose), photo_cloud_world)

            #mask = self._filter_cloud(photo_cloud_camera, photo_cloud_camera[..., 2] > 0)
            photo_valid_mask = (photo_cloud_camera[..., 2] > 0)
            crop_mask = crop_with_aabb(photo_cloud_container, container_aabb)
            mask = numpy.logical_and(photo_valid_mask, crop_mask)

            container_cloud_local = photo_cloud_container[mask]
            container_color = photo_aligned_color[mask]

            container_cloud_world = transform(container_pose, container_cloud_local)

            container_clouds.append(container_cloud_world)
            container_colors.append(container_color)

        # join the point clouds together
        container_cloud = numpy.vstack(container_clouds)
        container_color = numpy.vstack(container_colors)

        # save the point cloud for debugging
        self.store.multi_put({
            'point_cloud': container_cloud,
            'point_cloud_color': container_color,
            'timestamp': time()
        }, root='/debug/container/{}'.format(location))

        return container_cloud

    def _filter_cloud(self, cloud, valid_mask, count_threshold=1000, distance_threshold=0.005):
        #return numpy.ones(cloud.shape[:-1], dtype=numpy.bool)

        (labeled_cloud, label_count) = distance_label(cloud, valid_mask, distance_threshold)
        logger.debug('found {} connected components'.format(label_count))

        histogram = numpy.bincount(labeled_cloud.reshape((-1,)))
        (keep,) = numpy.where(histogram > count_threshold)

        # do not include the background label
        keep = keep[keep > 0]

        logger.debug('kept {} connected components'.format(len(keep)))

        return numpy.isin(labeled_cloud, keep)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ep')
    EvaluatePlacement(myname).run()
