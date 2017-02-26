import logging

from master.fsm import State
from master.world import build_world

from motion import planner

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanRoute(State):
    def run(self):
        item = self.store.get('/robot/selected_item')
        box = self.store.get('/robot/selected_box')

        logger.info('planning route for "{}" to "{}"'.format(item, box))

        world = build_world(self.store, ignore=['camera'])

        item_pose = self.store.get(['item', item, 'pose'])
        shelf_pose = self.store.get('/shelf/pose')

        target_item = {
            'position': list(shelf_pose.dot(item_pose)[:3, 3].flat),
            'vacuum_offset': [0, 0, 0.02],
            'drop offset': 0.3
        }

        box_pose = self.store.get(['box', box, 'pose'])

        target_box = {
            'position': box_pose[:3, 3].tolist(),
            'drop position': list((box_pose[:3, 3] + [0, 0, 0.3]).flat)
        }

        # compute route
        try:
            motion_plan = planner.pick_up(world, target_item, target_box)

            if not motion_plan:
                raise RuntimeError('motion plan is empty')
        except Exception:
            self.store.put('/status/route_plan', False)
            logger.exception('failed to generate pick motion plan')
        else:
            self.store.put('/robot/waypoints', motion_plan)
            self.store.put('/status/route_plan', True)
            logger.info('route generated')

if __name__ == '__main__':
    PlanRoute('pr').run()
