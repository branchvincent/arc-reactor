import logging

import numpy

from time import time

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

        world = build_world(self.store, ignore=['camera', 'items'])
        self.world = world

        item_pose_local = self.store.get(['item', item, 'pose'])
        item_pc_local = self.store.get(['item', item, 'point_cloud'])

        if location.startswith('bin'):
            reference_pose = self.store.get('/shelf/pose')
        elif location in ['stow_tote', 'stow tote']:
            reference_pose = self.store.get('/tote/stow/pose')
        else:
            raise RuntimeError('unrecognized item location: "{}"'.format(location))

        item_pose_world = reference_pose.dot(item_pose_local)
        item_pc_world = item_pc_local.dot(item_pose_world[:3, :3].T) + item_pose_world[:3, 3].T

        bounding_box = [
            [item_pc_world[:, 0].min(), item_pc_world[:, 1].min(), item_pc_world[:, 2].max()],
            [item_pc_world[:, 0].max(), item_pc_world[:, 1].max(), item_pc_world[:, 2].max()]
        ]
        logger.debug('item bounding box: {}'.format(bounding_box))
        # item_position = [item_pc_world[:, 0].mean(), item_pc_world[:, 1].mean(), item_pc_world[:, 2].max()]
        # logger.info('item center top: {}'.format(item_position))

        target_item = {
            # 'bbox': [item_position, item_position],
            'bbox': bounding_box,
            'vacuum_offset': [0, 0, -0.01],
            'drop offset': [0, 0, 0.1],
        }

        self.store.put('/robot/target_bounding_box', bounding_box)

        # compute route
        try:
            if self.store.get('/test/skip_planning', False):
                motion_plan = [(1,{'robot': self.store.get('/robot/current_config')})]
                logger.warn('skipped motion planning for testing')

            elif alg=='pick':
                box_pose = self.store.get(['box', box, 'pose'])

                target_box = {
                    'position': list(box_pose[:3, 3].flat),
                    'drop position': list(box_pose[:3, 3].flat)
                }

                logger.info('requesting pick motion plan')
                self.arguments = {'target_item': target_item, 'target_box': target_box}
                logger.debug('arguments\n{}'.format(self.arguments))

                motion_plan = planner.pick_up(world, target_item, target_box, 0)

            elif alg=='stow':
                # XXX: this will actually need to be a shelf location eventually...

                shelf_pose = self.store.get(['shelf', 'pose'])
                from master.world import xyz

                target_bin = self.store.get(['robot', 'selected_bin'])
                bin_pose_local = self.store.get(['shelf', 'bin', target_bin, 'pose'])
                bin_pose_world = shelf_pose.dot(bin_pose_local)

                bin_bounds_local = numpy.array(self.store.get(['shelf', 'bin', target_bin, 'bounds'])).T
                bin_bounds_world = bin_pose_world[:3,:3].dot(bin_bounds_local) + bin_pose_world[:3, 3]
                bin_target_world = numpy.matrix([bin_bounds_world[0].mean(), bin_bounds_world[1].mean(), bin_bounds_world[2].max()]).T

                target_box = {
                    'position': list(bin_target_world.flat),
                    'drop position': list(bin_target_world.flat)
                }

                logger.info('requesting stow motion plan')
                self.arguments = {'target_item': target_item, 'target_box': target_box}
                logger.debug('arguments\n{}'.format(self.arguments))

                motion_plan = planner.stow(world, target_item, target_box, 0)

            if not motion_plan:
                raise RuntimeError('motion plan is empty')
        except Exception:
            self.store.put('/status/route_plan', False)
            logger.exception('failed to generate motion plan')
        else:
            self.store.put('/robot/waypoints', motion_plan)
            self.store.put('/status/route_plan', True)
            self.store.put('/robot/timestamp', time())
            logger.info('route generated')

if __name__ == '__main__':
    PlanRoute('pr').run()
