import logging
import numpy
from time import time
from master.fsm import State
from master.world import build_world
from motion.new_planner import StowPlanner

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanStowGrab(State):
    def run(self):
        
        item = self.store.get('/robot/selected_item')
        grasp = self.store.get('/robot/selected_grasp')
        # item is in stow tote
        location = self.store.get(['item', item, 'location'])

        logger.info('planning route for "{}" to shelf'.format(item))

        world = build_world(self.store)
        self.world = world

        item_pose_local = self.store.get(['item', item, 'pose'])
        item_pc_local = self.store.get(['item', item, 'point_cloud'])

#        if location in ['stow_tote', 'stow tote']:
        reference_pose = self.store.get('/tote/stow/pose')
#        else:
#            raise RuntimeError('unrecognized item location: "{}"'.format(location))

        item_pose_world = reference_pose.dot(item_pose_local)
        item_pc_world = item_pc_local.dot(item_pose_world[:3, :3].T) + item_pose_world[:3, 3].T

        bounding_box = [
            [item_pc_world[:, 0].min(), item_pc_world[:, 1].min(), item_pc_world[:, 2].max()],
            [item_pc_world[:, 0].max(), item_pc_world[:, 1].max(), item_pc_world[:, 2].max()]
        ]
        logger.debug('item bounding box: {}'.format(bounding_box))
       
        target_item = {
            # 'bbox': [item_position, item_position],
            'bbox': bounding_box,
            'vacuum_offset': [0, 0, -0.01],
            'drop offset': [0, 0, 0.1],
        } #TODO make sure this info is in db, not stored locally here. why are these values selected?

        self.store.put('/robot/target_bounding_box', bounding_box)

        # compute route
        try:
            if self.store.get('/test/skip_planning', False):
                motion_plan = [(1,{'robot': self.store.get('/robot/current_config')})]
                logger.warn('skipped motion planning for testing')

            else:
                logger.info('requesting stow motion plan')
                self.arguments = {'target_item': target_item}
                logger.debug('arguments\n{}'.format(self.arguments))

                planner = StowPlanner(self.world, self.store)
                motion_plan = planner.stow_grab(target_item)

            if not motion_plan:
                raise RuntimeError('motion plan is empty')
        except Exception:
            self.setOutcome(False)
            logger.exception('Failed to generate motion plan')
        else:
            milestone_map = [m.get_milestone() for m in motion_plan]
            self.store.put('/robot/waypoints', milestone_map)
            self.setOutcome(True)
            self.store.put('/robot/timestamp', time())
            logger.info('Route generated')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'psg')
    PlanStowGrab(myname).run()
