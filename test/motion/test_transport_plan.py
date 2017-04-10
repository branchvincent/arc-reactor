import numpy

from test.pensive.helper import DatabaseDependentTestCase

from states.plan_route import PlanRoute

from master import workcell
from master.world import xyz

class PickTransportTest(DatabaseDependentTestCase):
    def setUp(self):
        super(PickTransportTest, self).setUp()
        self.store = self.client.default()

        # build a pick workcell
        workcell.setup_pick(self.store,
            workcell='db/workcell.json',
            location='db/item_location_file_pick.json',
            order='db/order_file.json'
        )

    def test_transport(self):
        item_name = self.store.get(['item']).keys()[0]
        self.store.put(['robot', 'selected_item'], item_name)

        # give the item a fake point cloud at its local origin
        self.store.put(['item', item_name, 'point_cloud'], numpy.array([[0, 0, 0]]))

        shelf_pose = self.store.get(['shelf', 'pose'])

        # move the object around in the bin
        for bin_name in self.store.get(['shelf', 'bin'], {}).keys():
            # get the bin bounding box
            bin_pose_local = self.store.get(['shelf', 'bin', bin_name, 'pose'])
            bin_bounds_local = numpy.array(self.store.get(['shelf', 'bin', bin_name, 'bounds']))

            # generate list of start points
            points = numpy.meshgrid(*[numpy.linspace(bin_bounds_local[0][i], bin_bounds_local[1][i], 5) for i in range(3)])
            points = numpy.hstack([p[1:-1, 1:-1, 1:-1].reshape((-1, 1)) for p in points])

            # plan routes for all of these points
            for point in points:
                self.store.put(['item', item_name, 'pose'], xyz(*list(point.flat)))

                # use each box as an endpoint
                for box_name in self.store.get(['box'], {}).keys():
                    self.store.put(['robot', 'selected_box'], box_name)

                    # try the route
                    self.store.put(['robot', 'current_config'], [0, 0.5526410543514846, -0.7175921219574686, 1.4195984536946278, 0.16760396806901545, 1.2698666571660342, 3.6207727997248367])
                    PlanRoute('pr', store=self.store).run()
                    self.assertTrue(self.store.get('/status/route_plan'))
