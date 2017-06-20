from master.fsm import State
from master.world import rpy, xyz
from motion.planner import MotionPlanner

import logging
import numpy
from time import time
from math import pi

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanStowGrab(State):
    """
    Input:
        - /robot/selected_item: name of item to be picked
        - /robot/target_grasp: dictionary containing item's grasp information
        - /robot/<selected_item>/location: location of item (NOTE: needed? sanity check only)
        - /robot/active_gripper: name of gripper to be used (vacuum or mechanical) (NOTE: not yet used)
    Output:
        - /robot/target_bounding_box: bounding box of target item (NOTE: not used?)
        - /robot/waypoints: list of milestones
        - /robot/timestamp: time of route generation
        - /failure/plan_pick_item: failure string
    Failure Cases:
        - infeasible: /robot/target_grasp is not a feasible grasp
    Dependencies:
        - selected_item
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

        # Get item location (must be stow tote)
        location = self.store.get(['item', item, 'location'])
        if location not in ['stow_tote', 'stow tote']:
            raise RuntimeError('/item/{}/location is not recognized: "{}"'.format(item, location))

        logger.info('planning route for "{}" to stow tote'.format(item))

        T_item = xyz(*grasp['center'][0])

        try:
            if self.store.get('/test/skip_planning', False):
                motion_plan = [(1, {'robot': self.store.get('/robot/current_config')})]
                logger.warn('skipped motion planning for testing')
            elif gripper == 'vacuum':
                logger.info('requesting stow motion plan')
                planner = MotionPlanner(self.store)
                motion_plan = planner.pick_to_inspect(T_item)
            else: #mechanical
                #TODO: develop planner for mechanical gripper
                raise NotImplementedError('Mechanical gripper planner does not exist')

            # Check motion plan
            if motion_plan is None:
                #TODO why did it fail? mark grasp unviable and move on
                failed_grasps = self.store.get('/grasp/failed_grasps', [])
                failed_grasps.append(grasp)
                self.store.put('/grasp/failed_grasps', failed_grasps)
                self.setOutcome(False)
                self.store.put('failure/plan_stow_grab', 'infeasible')
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
            #TODO why did it fail?

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'psg')
    PlanStowGrab(myname).run()
