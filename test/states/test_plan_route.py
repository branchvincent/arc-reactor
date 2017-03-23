from unittest import SkipTest

import json
import gzip

from test.pensive.helper import DatabaseDependentTestCase

from states.plan_route import PlanRoute

class PlanRoute_Direct(DatabaseDependentTestCase):
    def setUp(self):
        super(PlanRoute_Direct, self).setUp()

        try:
            import klampt
        except ImportError:
            raise SkipTest('Klampt is not installed')

        self.store = self.client.default()
        self.store.put('', json.loads(gzip.open('data/test/test_031017_new_pick.json.gz').read()))

        self.pr = PlanRoute('pr1', store=self.store)

    def test_goDirect(self):
        self.pr.run()
        self.assertTrue(self.store.get('/status/route_plan', True))

    def tearDown(self):
        pass
