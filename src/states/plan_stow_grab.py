from master.fsm import State
from master.world import build_world, rpy
from motion.new_planner import StowPlanner
from util.math_helpers import build_pose, transform, rotate, normalize

import logging
import numpy
from time import time
from math import pi

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanStowGrab(State):
    def run(self):

        # Get item and grasp info
        item = self.store.get('/robot/selected_item')
        grasp = self.store.get('/robot/target_grasp')
        # TODO: add to db (defaults to vacuum for now)
        gripper = self.store.get('/robot/active_gripper', 'vacuum').lower()
        if gripper not in ['vacuum', 'mechanical']:
            raise RuntimeError('unrecognized gripper "{}"'.format(gripper))

        logger.info('planning route for "{}" to stow tote'.format(item))

        # Get item location (must be stow tote)
        location = self.store.get(['item', item, 'location'])
        if location not in ['stow_tote', 'stow tote']:
            raise RuntimeError('unrecognized item location: "{}"'.format(location))
        reference_pose = self.store.get('/tote/stow/pose')

        # Calculate item bounding box
        # NOTE: single point for now
        self.world = build_world(self.store)
        bounding_box = [grasp['center'][0], grasp['center'][0]]
        # item_pose_local = self.store.get(['item', item, 'pose'])
        # item_pc_local = self.store.get(['item', item, 'point_cloud'])
        #
        # item_pose_world = reference_pose.dot(item_pose_local)
        # item_pc_world = item_pc_local.dot(item_pose_world[:3, :3].T) + item_pose_world[:3, 3].T
        #
        # bounding_box = [
        #     [item_pc_world[:, 0].min(), item_pc_world[:, 1].min(), item_pc_world[:, 2].max()],
        #     [item_pc_world[:, 0].max(), item_pc_world[:, 1].max(), item_pc_world[:, 2].max()]
        # ]
        self.store.put('/robot/target_bounding_box', bounding_box)
        logger.debug('item bounding box: {}'.format(bounding_box))

        # Construct arguments for planner
        target_item = {
            # 'bbox': [item_position, item_position],
            'bbox': bounding_box,
            'vacuum_offset': [0, 0, -0.01],
            'drop offset': [0, 0, 0.1],
        } #TODO make sure this info is in db, not stored locally here. why are these values selected?

        # Compute pose
        # pose = numpy.eye(4)
        # # normal vector points along Z
        # pose[:3, 2] = normalize(grasp['orientation'])
        # pose[:3, 0] = normalize(numpy.cross(rotate(rpy(pi / 2, 0, 0), pose[:3, 2]), pose[:3, 2]))
        # pose[:3, 1] = normalize(numpy.cross(pose[:3, 2], pose[:3, 0]))
        # # position is grasp center
        # pose[:3, 3] = grasp['center']

        # compute route
        try:
            if self.store.get('/test/skip_planning', False):
                motion_plan = [(1, {'robot': self.store.get('/robot/current_config')})]
                logger.warn('skipped motion planning for testing')
            elif gripper == 'vacuum':
                logger.info('requesting stow motion plan')
                self.arguments = {'target_item': target_item}
                logger.debug('arguments\n{}'.format(self.arguments))
                planner = StowPlanner(self.world, self.store)
                motion_plan = planner.stow_grab(target_item)
            else: #mechanical
                #TODO: develop planner for mechanical gripper
                raise NotImplementedError('Mechanical gripper planner does not exist')

            # Check motion plan
            if motion_plan is None:
                self.setOutcome(False)
                raise RuntimeError('motion plan is empty')
            else:
                milestone_map = [m.get_milestone() for m in motion_plan]
                self.store.put('/robot/waypoints', milestone_map)
                self.store.put('/robot/timestamp', time())
                self.setOutcome(True)
                logger.info('Route generated')
        except Exception:
            self.setOutcome(False)
            logger.exception('Failed to generate motion plan')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'psg')
    PlanStowGrab(myname).run()
