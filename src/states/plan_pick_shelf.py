from master.fsm import State
from master.world import xyz, rpy
from motion.planner import MotionPlanner
from util.math_helpers import build_pose, transform, rotate, normalize

import logging
import numpy
from time import time
from math import pi

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanPickShelf(State):
    """
    Input:
        - /robot/selected_item: name of item to be picked
        - /robot/target_grasp: dictionary containing item's grasp information
        - /robot/active_gripper: name of gripper to be used (vacuum or mechanical) (NOTE: not yet used)
    Output:
        - /robot/target_bounding_box: bounding box of target item (NOTE: not used?)
        - /robot/waypoints: list of milestones
        - /robot/timestamp: time of route generation
        - /failure/plan_pick_shelf: failure string
    Failure Cases:
        - infeasible: /robot/target_grasp is not a feasible grasp
    Dependencies:
        - select_item
    """

    def run(self):
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

        logger.info('planning route for "{}" from "{}"'.format(item, grasp['location']))

        # T_item = xyz(*grasp['center'][0])
        # normal = grasp['orientation'].tolist()[0]

        T_item = numpy.eye(4)
        # normal vector points along Z
        T_item[:3, 2] = normalize(grasp['orientation'])
        T_item[:3, 0] = normalize(numpy.cross(rotate(rpy(pi / 2, 0, 0), T_item[:3, 2]), T_item[:3, 2]))
        T_item[:3, 1] = normalize(numpy.cross(T_item[:3, 2], T_item[:3, 0]))
        # position is grasp center
        T_item[:3, 3] = grasp['center']

        # compute route
        try:
            if self.store.get('/test/skip_planning', False):
                motion_plan = [(1, {'robot': self.store.get('/robot/current_config')})]
                logger.warn('skipped motion planning for testing')
            elif gripper == 'vacuum':
                logger.info('requesting pick motion plan')
                planner = MotionPlanner(store=self.store)
                motion_plan = planner.pickToInspect(T_item)
            else: #mechanical
                #TODO: develop planner for mechanical gripper
                raise NotImplementedError('Mechanical gripper planner does not exist')

            # Check motion plan
            if motion_plan is None:
                failed_grasps = self.store.get('/grasp/failed_grasps', [])
                failed_grasps.append(grasp)
                self.store.put('/grasp/failed_grasps', failed_grasps)
                self.setOutcome(False)
                self.store.put('failure/plan_pick_shelf', 'infeasible')
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
    myname = (args.name or 'ppi')
    PlanPickShelf(myname).run()
