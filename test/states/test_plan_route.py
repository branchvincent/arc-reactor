from unittest import TestCase, SkipTest
from states.plan_route import PlanRoute

class PlanRoute_Direct(TestCase):
    def setUp(self):
        self.pr = PlanRoute('pr1')
        #default store

    def test_goDirect(self):
        self.pr.run()
        self.assertEqual(self.pr.store.get('/robot/current_config'), self.pr.store.get('/robot/goal_config'))

    def tearDown(self):
        pass
