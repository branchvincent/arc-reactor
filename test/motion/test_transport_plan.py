import numpy

from test.pensive.helper import DatabaseDependentTestCase

from states.plan_route import PlanRoute

from master import workcell
from master.world import xyz

def _call_with_self(target, *args, **kwargs):
    def _cb(self):
        return target(self, *args, **kwargs)
    return _cb

def _generate_pick_tests():
    def setUp(self):
        super(self.__class__, self).setUp()
        self.store = self.client.default()

        # build a pick workcell
        workcell.setup_pick(self.store,
            workcell='db/workcell.json',
            location='db/item_location_file_pick.json',
            order='db/order_file.json'
        )

    def handle_test(self, bin_index, density, point_index, box_index):
        item_name = self.store.get(['item']).keys()[0]
        self.store.put(['robot', 'selected_item'], item_name)

        # give the item a fake point cloud at its local origin
        self.store.put(['item', item_name, 'point_cloud'], numpy.array([[0, 0, 0]]))

        shelf_pose = self.store.get(['shelf', 'pose'])

        # move the object around in the bin
        bin_name = self.store.get(['shelf', 'bin']).keys()[bin_index]
        # get the bin bounding box
        bin_pose_local = self.store.get(['shelf', 'bin', bin_name, 'pose'])
        bin_bounds_local = numpy.array(self.store.get(['shelf', 'bin', bin_name, 'bounds']))

        # generate list of start points
        points = numpy.meshgrid(*[numpy.linspace(bin_bounds_local[0][i], bin_bounds_local[1][i], density + 2) for i in range(3)])
        points = numpy.hstack([p[1:-1, 1:-1, 1:-1].reshape((-1, 1)) for p in points])

        # plan routes for all of these points
        point = points[point_index]
        self.store.put(['item', item_name, 'pose'], xyz(*list(point.flat)))

        # use each box as an endpoint
        box_name = self.store.get(['box']).keys()[box_index]
        self.store.put(['robot', 'selected_box'], box_name)

        # try the route
        self.store.put(['robot', 'current_config'], [0, 0.5526410543514846, -0.7175921219574686, 1.4195984536946278, 0.16760396806901545, 1.2698666571660342, 3.6207727997248367])
        PlanRoute('pr', store=self.store).run()
        self.assertTrue(self.store.get('/status/route_plan'))


    methods = {
        'setUp': setUp,
    }

    density = 1

    for bin_index in range(3):
        for point_index in range(density**3):
            for box_index in range(3):
                methods['test_pick_{}{}{}'.format(bin_index, point_index, box_index)] = _call_with_self(handle_test, bin_index, density, point_index, box_index)

    return type('PickTransportTest', (DatabaseDependentTestCase,), methods)

PickTransportTest = _generate_pick_tests()

# def _generate_stow_tests():
#     def setUp(self):
#         super(self.__class__, self).setUp()
#         self.store = self.client.default()

#         # build a pick workcell
#         workcell.setup_stow(self.store,
#             workcell='db/workcell.json',
#             location='db/item_location_file_pick.json'
#         )

#     def handle_test(self, bin_index, density, point_index, box_index):
#         item_name = self.store.get(['item']).keys()[0]
#         self.store.put(['robot', 'selected_item'], item_name)

#         # give the item a fake point cloud at its local origin
#         self.store.put(['item', item_name, 'point_cloud'], numpy.array([[0, 0, 0]]))

#         box_pose = self.store.get(['shelf', 'pose'])

#         # move the object around in the bin
#         bin_name = self.store.get(['shelf', 'bin']).keys()[bin_index]
#         # get the bin bounding box
#         bin_pose_local = self.store.get(['shelf', 'bin', bin_name, 'pose'])
#         bin_bounds_local = numpy.array(self.store.get(['shelf', 'bin', bin_name, 'bounds']))

#         # generate list of start points
#         points = numpy.meshgrid(*[numpy.linspace(bin_bounds_local[0][i], bin_bounds_local[1][i], density + 2) for i in range(3)])
#         points = numpy.hstack([p[1:-1, 1:-1, 1:-1].reshape((-1, 1)) for p in points])

#         # plan routes for all of these points
#         point = points[point_index]
#         self.store.put(['item', item_name, 'pose'], xyz(*list(point.flat)))

#         # use each box as an endpoint
#         box_name = self.store.get(['box']).keys()[box_index]
#         self.store.put(['robot', 'selected_box'], box_name)

#         # try the route
#         self.store.put(['robot', 'current_config'], [0, 0.5526410543514846, -0.7175921219574686, 1.4195984536946278, 0.16760396806901545, 1.2698666571660342, 3.6207727997248367])
#         PlanRoute('pr', store=self.store).run()
#         self.assertTrue(self.store.get('/status/route_plan'))


#     methods = {
#         'setUp': setUp,
#     }

#     density = 3

#     for bin_index in range(3):
#         for point_index in range(density**3):
#             for box_index in range(3):
#                 methods['test_pick_{}{}{}'.format(bin_index, point_index, box_index)] = _call_with_self(handle_test, bin_index, density, point_index, box_index)

#     return type('StowTransportTest', (DatabaseDependentTestCase,), methods)

# StowTransportTest = _generate_stow_tests()
