from unittest import SkipTest

from test.pensive.helper import DatabaseDependentTestCase

from states.plan_route import PlanRoute

import json
with open('test/master/test_022717_badik2.json') as data_file:
    initial_db = json.load(data_file)

class PlanRoute_Direct(DatabaseDependentTestCase):
    def setUp(self):
        super(PlanRoute_Direct, self).setUp()

        try:
            import klampt
        except ImportError:
            raise SkipTest('Klampt is not installed')

        self.store = self.client.default()
        self.store.put('', initial_db)

        self.pr = PlanRoute('pr1', store=self.store)
        #default store

    def test_goDirect(self):
        self.pr.run()
        self.assertTrue(self.store.get('/status/route_plan', True))

    def tearDown(self):
        pass
