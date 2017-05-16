import logging

import numpy

from time import time

from master.fsm import State
from master.world import build_world

from motion.new_planner import PickPlanner

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanPutItem(State):
    def run(self):
        item = self.store.get('/robot/selected_item')
        box = self.store.get('/robot/selected_box')
        alg = self.store.get('/robot/task')
        print "item is ", item, " in box ", box

        if not alg: raise RuntimeError("Task undefined")

        location = self.store.get(['item', item, 'location'])

        logger.info('planning route for "{}" to "{}"'.format(item, box))

        world = build_world(self.store)
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
        } #TODO make sure this info is in db, not stored locally here

        self.store.put('/robot/target_bounding_box', bounding_box)

        # compute route
        try:
            if self.store.get('/test/skip_planning', False):
                motion_plan = [(1,{'robot': self.store.get('/robot/current_config')})]
                logger.warn('skipped motion planning for testing')

            elif alg=='pick':

                planner = PickPlanner(self.world, self.store) 

                box_pose = self.store.get(['box', box, 'pose'])

                target_box = {
                    'name': box,
                    'position': list(box_pose[:3, 3].flat),
                    'drop position': list(box_pose[:3, 3].flat)
                }
                #target_box["box_limit"]=world.rigidObject('{}_box'.format(box)).geometry().getBB()
                target_box["box_limit"] = self.store.get('/box/'+box+'/bounds')

                logger.info('requesting pick motion plan')
                self.arguments = {'target_item': target_item, 'target_box': target_box}
                logger.debug('arguments\n{}'.format(self.arguments))

                #assume already near item (?)
                motion_plan = planner.drop_item(target_item, target_box, 'item_{}'.format(item))

            elif alg=='stow':
                # XXX: this will actually need to be a shelf location eventually...

                planner = PickPlanner(self.world, self.store) 

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
                #target_box["box_limit"]=[[-0.2144207123374089, 0.55756321667681634, 0.02],[0.05442071233740891, 0.7024367833231837, 0.42065000000000001]]
               
                logger.info('requesting stow motion plan')
                self.arguments = {'target_item': target_item, 'target_box': target_box}
                logger.debug('arguments\n{}'.format(self.arguments))

                motion_plan = planner.stow(target_item, target_box, "item_"+item)

            if not motion_plan:
                raise RuntimeError('motion plan is empty')
        except Exception:
            self.store.put('/status/route_plan_put', False)
            logger.exception('Failed to generate motion plan')
        else:
            milestone_map = [m.get_milestone() for m in motion_plan]
            self.store.put('/robot/waypoints', milestone_map)
            self.store.put('/status/route_plan_put', True)
            self.store.put('/robot/timestamp', time())
            logger.info('Route generated')

if __name__ == '__main__':
    PlanPutItem('ppi').run()
