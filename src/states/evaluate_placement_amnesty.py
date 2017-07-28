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

class EvaluatePlacementAmnesty(State):
    '''
    Inputs:
     - /robot/target_locations (e.g., ['binA'])
     - photos for each target location

    Outputs:
     - /robot/placement/pose: a 4x4 numpy matrix of end-effector pose
     - /robot/placement/location: location name of placement
     - /robot/target_bin and /robot/target_location: set to location of placement

    Failures:
     - None

    Dependencies:
     -
    '''

    def run(self):
        locations = self.store.get('/robot/target_locations', [])
        logger.info('finding placement in {}'.format(locations))

        try:
            self._handler()
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

    def _handler(self):
        logger.warn('using hardcoded amnesty drop site')

        idx = 0
        position = self.store.get('/robot/amnesty_drop_position')
        orientation = 0

        # packing succeeded
        pack_location = 'amnesty_tote'
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

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'epa')
    EvaluatePlacementAmnesty(myname).run()
