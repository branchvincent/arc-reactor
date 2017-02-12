from test.pensive.helper import DatabaseDependentTestCase

from states.plan_route import PlanRoute

class PlanRoute_Direct(DatabaseDependentTestCase):
    def setUp(self):
        super(PlanRoute_Direct, self).setUp()

        self.pr = PlanRoute('pr1', store=self.client.default())
        #default store

    def test_goDirect(self):
        self.pr.run()
        self.assertEqual(self.pr.store.get('/robot/current_config'), self.pr.store.get('/robot/goal_config'))

    def tearDown(self):
        pass
