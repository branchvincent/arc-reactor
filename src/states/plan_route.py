import logging

from master.fsm import State
from master.world import build_world

from motion import planner

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanRoute(State):
    def run(self):
        item = self.store.get('/robot/selected_item')
        box = self.store.get('/robot/selected_box')
        alg = self.store.get('/robot/task')
        
        if not alg: raise RuntimeError("Task undefined")           

        logger.info('planning route for "{}" to "{}"'.format(item, box))

        world = build_world(self.store, ignore=['camera'])

        item_pose_local = self.store.get(['item', item, 'pose'])
        shelf_pose = self.store.get('/shelf/pose')

        item_pc_local = self.store.get(['item', item, 'point_cloud'])
        item_pose_world = shelf_pose.dot(item_pose_local)
        item_pc_world = item_pc_local.dot(item_pose_world[:3, :3]) + item_pose_world[:3, 3].T

        item_position = [item_pc_world[:, 0].mean(), item_pc_world[:, 1].mean(), item_pc_world[:, 2].max()]
        logger.info('item center top: {}'.format(item_position))

        target_item = {
            'bbox': [item_position, item_position],
            'vacuum_offset': [0, 0, 0.02],
            'drop offset': 0.40
        }

        box_pose = self.store.get(['box', box, 'pose'])

        target_box = {
            'position': list(box_pose[:3, 3].flat),
            'drop position': list((box_pose[:3, 3] + [[0], [0], [0.4]]).flat)
        }

        # compute route
        try:
            if alg=='pick':
                motion_plan = planner.pick_up(world, target_item, target_box)
            elif alg=='stow':
                motion_plan = None

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
