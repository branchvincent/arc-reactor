import logging
import numpy
from time import time
from master.fsm import State
from master.world import build_world
from motion.linear_planner import Planner

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

        # Get item location (must be stow tote)
        # location = self.store.get(['item', item, 'location'])
        # if location not in ['stow_tote', 'stow tote']:
        #     raise RuntimeError('unrecognized item location: "{}"'.format(location))
        # reference_pose = self.store.get('/tote/stow/pose')

        # Calculate item bounding box
        # NOTE: single point for now
        # self.world = build_world(self.store)
        # bounding_box = [grasp['center'][0], grasp['center'][0]]
        # self.store.put('/robot/target_bounding_box', bounding_box)
        # logger.debug('item bounding box: {}'.format(bounding_box))

        # Construct arguments for planner
        # target_item = {
        #     # 'bbox': [item_position, item_position],
        #     # 'bbox': bounding_box,
        #     'vacuum_offset': [0, 0, -0.01],
        #     'drop offset': [0, 0, 0.05],
        # } #TODO make sure this info is in db, not stored locally here. why are these values selected?

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
                # self.arguments = {'target_item': target_item}
                # logger.debug('arguments\n{}'.format(self.arguments))
                # planner = StowPlanner(self.world, self.store)
                # motion_plan = planner.place_shelf(target_item, target_T)
                planner = Planner(self.store)
                motion_plan = planner.place_shelf(target_T)
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
