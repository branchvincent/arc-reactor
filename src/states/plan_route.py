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

        location = self.store.get(['item', item, 'location'])

        logger.info('planning route for "{}" to "{}"'.format(item, box))

        world = build_world(self.store, ignore=['camera'])

        item_pose_local = self.store.get(['item', item, 'pose'])
        item_pc_local = self.store.get(['item', item, 'point_cloud'])

        if location == 'shelf':
            reference_pose = self.store.get('/shelf/pose')
        elif location in ['stow_tote', 'stow tote']:
            reference_pose = self.store.get('/tote/stow/pose')
        else:
            raise RuntimeError('unrecognized item location: "{}"'.format(location))

        item_pose_world = reference_pose.dot(item_pose_local)
        item_pc_world = item_pc_local.dot(item_pose_world[:3, :3]) + item_pose_world[:3, 3].T

        bounding_box = [
            [item_pc_world[:, 0].min(), item_pc_world[:, 1].min(), item_pc_world[:, 2].max()],
            [item_pc_world[:, 0].max(), item_pc_world[:, 1].max(), item_pc_world[:, 2].max()]
        ]
        logger.info('item bounding box: {}'.format(bounding_box))
        # item_position = [item_pc_world[:, 0].mean(), item_pc_world[:, 1].mean(), item_pc_world[:, 2].max()]
        # logger.info('item center top: {}'.format(item_position))

        target_item = {
            # 'bbox': [item_position, item_position],
            'bbox': bounding_box,
            'vacuum_offset': [0, 0, 0.02],
            'drop offset': 0.20
        }

        # compute route
        try:
            if alg=='pick':
                box_pose = self.store.get(['box', box, 'pose'])

                target_box = {
                    'position': list(box_pose[:3, 3].flat),
                    'drop position': list((box_pose[:3, 3] + [[0], [0], [0.4]]).flat)
                }

                motion_plan = planner.pick_up(world, target_item, target_box)
            elif alg=='stow':
                # XXX: this will actually need to be a shelf location eventually...

                shelf_pose = self.store.get(['shelf', 'pose'])
                from master.world import xyz

                bin_pose = shelf_pose * xyz(0, 0, 0.6)

                target_box = {
                    'position': list(bin_pose[:3, 3].flat),
                    'drop position': list((bin_pose[:3, 3] + [[0], [0], [0.2]]).flat)
                }

                motion_plan = planner.stow(world, target_item, target_box)

            if not motion_plan:
                raise RuntimeError('motion plan is empty')
        except Exception:
            self.store.put('/status/route_plan', False)
            logger.exception('failed to generate motion plan')
        else:
            self.store.put('/robot/waypoints', motion_plan)
            self.store.put('/status/route_plan', True)
            logger.info('route generated')

if __name__ == '__main__':
    PlanRoute('pr').run()
