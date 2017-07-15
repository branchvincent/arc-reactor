from master.fsm import State
from master.world import build_world
from motion.planner import MotionPlanner

import logging
import numpy
from time import time

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanStowBase(State):
    def _common(self):
        # Get inputs
        item = self.store.get('/robot/selected_item')
        target_pose = self.store.get(['robot', 'placement', 'pose'])
        target_location = self.store.get(['robot', 'placement', 'location'])
        # TODO: add to db
        gripper = self.store.get('/robot/active_gripper', 'vacuum').lower()

        # Check inputs
        if item is None:
            raise RuntimeError('/robot/selected_item is none')
        elif target_pose is None:
            raise RuntimeError('/robot/placement/pose is none')
        elif target_location is None:
            raise RuntimeError('/robot/placement/location is none')
        elif gripper not in ['vacuum', 'mechanical']:
            raise RuntimeError('/robot/active_gripper is not unrecognized: "{}"'.format(gripper))

        logger.info('planning stow route for "{}" to "{}"'.format(item, target_location))

        # compute route
        if gripper == 'vacuum':
            planner = MotionPlanner(self.store)
            planner.stow(target_pose)
        else: #mechanical
            #TODO: develop planner for mechanical gripper
            raise NotImplementedError('Mechanical gripper planner does not exist')

        # Check motion plan
        motion_plan = self.store.get('/robot/waypoints')
        if motion_plan is None:
            logger.error('Failed to generate motion plan')
            self.store.put(['failure', self.getFullName()], 'infeasible')
            self.setOutcome(False)
        else:
            logger.info('Route generated')
            self.setOutcome(True)
