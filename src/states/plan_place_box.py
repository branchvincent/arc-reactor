import logging
import numpy
from time import time
from master.fsm import State
from master.world import build_world
from motion.planner import MotionPlanner

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanPlaceBox(State):
    """
    Input:
        - /robot/selected_item: name of item to be placed
        - /robot/placement/pose: end-effector's pose for item placement
        - /robot/placement/location: name of target box (NOTE: needed?)
        - /robot/active_gripper: gripper to be used (vacuum or mechanical) (NOTE: not yet used)
    Output:
        - /failure/plan_place_box: failure string
    Failure Cases:
        - infeasible: /robot/placement/pose is not a feasible placement
    Dependencies:
        - evaluate_placement
    """

    def run(self):
        # Get inputs
        item = self.store.get('/robot/selected_item')
        target_T = self.store.get(['robot', 'placement', 'pose'])
        target_location = self.store.get(['robot', 'placement', 'location'])
        # TODO: add to db
        gripper = self.store.get('/robot/active_gripper', 'vacuum').lower()

        # Check inputs
        if item is None:
            raise RuntimeError('/robot/selected_item is none')
        elif target_T is None:
            raise RuntimeError('/robot/placement/pose is none')
        elif target_location is None:
            raise RuntimeError('/robot/placement/location is none')
        elif gripper not in ['vacuum', 'mechanical']:
            raise RuntimeError('/robot/active_gripper is not unrecognized: "{}"'.format(gripper))

        logger.info('planning route for "{}" to "{}"'.format(item, target_location))

        # compute route
        if gripper == 'vacuum':
            logger.info('requesting pick motion plan')
            planner = MotionPlanner(self.store)
            motion_plan = planner.inspectToPlace(target_T)
        else: #mechanical
            #TODO: develop planner for mechanical gripper
            raise NotImplementedError('Mechanical gripper planner does not exist')

        # Check motion plan
        if self.store.get('/robot/waypoints') is None:
            logger.exception('Failed to generate motion plan')
            self.store.put('/failure/plan_place_box', 'infeasible')
            self.setOutcome(False)
        else:
            logger.info('Route generated')
            self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ppb')
    PlanPlaceBox(myname).run()
