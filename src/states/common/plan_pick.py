
from klampt.math import so3

from master.fsm import State
from master.world import xyz, rpy, klampt2numpy
from motion.planner import MotionPlanner, PlannerFailure
from util.math_helpers import build_pose, transform, rotate, normalize

import logging
import numpy
from time import time
from math import pi, acos

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanPickBase(State):
    def _common(self, inspect=True, lift=True):
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

        max_downshift = 0.01
        for scale in numpy.linspace(1, 0, 5):
            for sign in [1, -1]:
                normal = sign * normalize(grasp['orientation'])
                downshift = max_downshift * (1 - scale)

                normal = normalize((scale * normal + [0, 0, 1 - scale]) / 2)

                # Compute item pose
                T_item = numpy.eye(4)
                # normal vector points along Z
                T_item[:3, 2] = normalize(normal)
                T_item[:3, 0] = normalize(numpy.cross(rotate(rpy(pi / 2, 0, 0), T_item[:3, 2]), T_item[:3, 2]))
                T_item[:3, 1] = normalize(numpy.cross(T_item[:3, 2], T_item[:3, 0]))
                # position is grasp center
                T_item[:3, 3] = grasp['center']
                T_item[:3, 2] -= max_downshift

                try:
                    if gripper == 'vacuum':
                        planner = MotionPlanner(store=self.store)
                        if inspect:
                            planner.pickToInspect(T_item)
                        elif lift:
                            planner.pickLiftOnly(T_item)
                        else:
                            planner.pick(T_item)
                    else: #mechanical
                        #TODO: develop planner for mechanical gripper
                        raise NotImplementedError('Mechanical gripper planner does not exist')

                    break

                except PlannerFailure:
                    pass

                logger.warn('trying other normal')

            # Check motion plan
            motion_plan = self.store.get('/robot/waypoints')
            if motion_plan is not None:
                logger.info('Route generated')
                self.store.put('/status/ppo_done', False)
                self.setOutcome(True)
                break

            logger.warn('trying scale {}'.format(scale))

        else:
            self.store.put(['failure', self.getFullName()], 'infeasible')
            self.setOutcome(False)
            logger.error('Failed to generate motion plan')
