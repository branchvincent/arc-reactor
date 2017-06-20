import logging
import numpy
from time import time
from master.fsm import State
from master.world import build_world
from motion.planner import MotionPlanner

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanPlaceShelf(State):
    """
    Input:
        - /robot/selected_item: name of item to be placed
        - /robot/placement/pose: end-effector's pose for item placement
        - /robot/placement/location: name of target box (NOTE: needed?)
        - /robot/active_gripper: gripper to be used (vacuum or mechanical) (NOTE: not yet used)
    Output:
        - /failure/plan_place_shelf: failure string
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

        try:
            if self.store.get('/test/skip_planning', False):
                motion_plan = [(1, {'robot': self.store.get('/robot/current_config')})]
                logger.warn('skipped motion planning for testing')
            elif gripper == 'vacuum':
                logger.info('requesting stow motion plan')
                planner = MotionPlanner(self.store)
                motion_plan = planner.placeToInspect(target_T)
            else: #mechanical
                #TODO: develop planner for mechanical gripper
                raise NotImplementedError('Mechanical gripper planner does not exist')

            # Check motion plan
            if motion_plan is None:
                self.setOutcome(False)
                self.store.put('/failure/plan_place_shelf', 'infeasible')
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
    myname = (args.name or 'pps')
    PlanPlaceShelf(myname).run()
