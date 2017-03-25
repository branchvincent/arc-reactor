from unittest import SkipTest

import numpy

import json
import gzip

from test.pensive.helper import DatabaseDependentTestCase

from master.pickfsm import PickStateMachine

class SimplePickFSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimplePickFSM, self).setUp()

        try:
            import klampt
        except ImportError:
            raise SkipTest('Klampt is not installed')

        self.store = self.client.default()
        self.store.put('', json.loads(gzip.open('data/test/workcell_032517.json.gz').read()))

        self.store.put('/status/task', 'pick')

        # simulate for now
        self.store.put('/simulate/robot_motion', True)
        self.store.put('/simulate/object_detection', True)
        self.store.put('/simulate/cameras', True)

        # skip difficult to simulate parts
        self.store.put('/test/skip_grabcut', True)
        self.store.put('/test/skip_planning', True)

        self.pick = PickStateMachine(store=self.store)
        self.pick.loadOrderFile('test/master/order_test.json')
        self.pick.loadStates()
        self.pick.setupTransitions()

    def test_runPickOrder(self):
        self.pick.runOrdered('si')
        self.assertEqual(self.pick.getAllPastEvents(), ['SI', 'FI', 'PR', 'ER', 'CI'])

    #def test_runFullOrder(self):
    #    for _ in range(10): self.pick.runOrdered('si')

    def tearDown(self):
        pass
