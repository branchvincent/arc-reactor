from master.fsm import State
from master.world import xyz, rpy
from motion.planner import MotionPlanner
from util.math_helpers import build_pose, transform, rotate, normalize

import logging
import numpy
from time import time
from math import pi

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanPickBase(State):
    def _common(self, inspect=True):
        # Get inputs
        item = self.store.get('/robot/selected_item')
        grasp = self.store.get('/robot/target_grasp')
        # TODO: add to db
        gripper = self.store.get('/robot/active_gripper', 'vacuum').lower()

        # Check inputs
        if item is None:
            raise RuntimeError('/robot/selected_item is none')
        elif grasp is None:
            raise RuntimeError('/robot/target_grasp is none')
        elif gripper not in ['vacuum', 'mechanical']:
            raise RuntimeError('/robot/active_gripper is not unrecognized: "{}"'.format(gripper))

        # Plan route
        logger.info('planning pick route for "{}" from "{}"'.format(item, grasp['location']))

        for sign in [1, -1]:
            normal = [sign * x for x in grasp['orientation']]

            # Compute item pose
            T_item = numpy.eye(4)
            # normal vector points along Z
            T_item[:3, 2] = normalize(normal)
            T_item[:3, 0] = normalize(numpy.cross(rotate(rpy(pi / 2, 0, 0), T_item[:3, 2]), T_item[:3, 2]))
            T_item[:3, 1] = normalize(numpy.cross(T_item[:3, 2], T_item[:3, 0]))
            # position is grasp center
            T_item[:3, 3] = grasp['center']

            try:
                if gripper == 'vacuum':
                    planner = MotionPlanner(store=self.store)
                    if inspect:
                        planner.pickToInspect(T_item)
                    else:
                        planner.pick(T_item)
                else: #mechanical
                    #TODO: develop planner for mechanical gripper
                    raise NotImplementedError('Mechanical gripper planner does not exist')

                # Check motion plan
                motion_plan = self.store.get('/robot/waypoints')
                if motion_plan is not None:
                    logger.info('Route generated')
                    self.store.put('/status/ppo_done', False)
                    self.setOutcome(True)
                    break

            except PlannerFailure:
                logger.warn('trying other normal')
                pass

        else:
            self.store.put(['failure', self.getFullName()], 'infeasible')
            self.setOutcome(False)
            logger.error('Failed to generate motion plan')
