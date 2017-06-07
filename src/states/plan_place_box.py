import logging
import numpy
from time import time
from master.fsm import State
from master.world import build_world
from motion.new_planner import PickPlanner

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanPlaceBox(State):

    def run(self):
        # Get item info
        item = self.store.get('/robot/selected_item')
        # TODO: add to db (defaults to vacuum for now)
        gripper = self.store.get('/robot/active_gripper', 'vacuum').lower()
        if gripper not in ['vacuum', 'mechanical']:
            raise RuntimeError('unrecognized gripper "{}"'.format(gripper))

        # Get target location (must be a box)
        box = self.store.get('/robot/target_box')
        if not box.startswith('box'):
            raise RuntimeError('unrecognized box: "{}"'.format(box))
        reference_pose = self.store.get(['box', box, 'pose'])
        reference_bounds = self.store.get(['box', box, 'bounds'])

        logger.info('planning route for "{}" to "{}"'.format(item, box))

        # Calculate item bounding box
        # NOTE: single point for now
        self.world = build_world(self.store)
        # bounding_box = [grasp['center'][0]] * 2
        # self.store.put('/robot/target_bounding_box', bounding_box)
        # logger.debug('item bounding box: {}'.format(bounding_box))

        # Construct arguments for planner
        target_item = {
            # 'bbox': bounding_box,
            'vacuum_offset': [0, 0, -0.01],
            'drop offset': [0, 0, 0.1],
            'box_limit': reference_bounds
        } #TODO make sure this info is in db, not stored locally here. why are these values selected?

        target_box = {
            'name': box,
            'position': list(reference_pose[:3, 3].flat),
            'drop position': list(reference_pose[:3, 3].flat)
        }

        # compute route
        try:
            if self.store.get('/test/skip_planning', False):
                motion_plan = [(1, {'robot': self.store.get('/robot/current_config')})]
                logger.warn('skipped motion planning for testing')
            elif gripper == 'vacuum':
                logger.info('requesting pick motion plan')
                self.arguments = {'target_item': target_item, 'target_box': target_box}
                logger.debug('arguments\n{}'.format(self.arguments))
                planner = PickPlanner(self.world, self.store)
                #assume already have item. moving to put item in box
                motion_plan = planner.drop_item(target_item, target_box, 'item_{}'.format(item))
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
    myname = (args.name or 'ppb')
    PlanPlaceBox(myname).run()
