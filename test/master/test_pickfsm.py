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
        self.store.put('', json.loads(gzip.open('data/test/workcell_032617.json.gz').read()))

        self.store.put('/status/task', 'pick')

        # simulate for now
        self.store.put('/simulate/robot_motion', True)
        self.store.put('/simulate/object_detection', True)
        self.store.put('/simulate/cameras', True)

        # disable checkpoints
        self.store.delete('/checkpoint')

        # skip difficult to simulate parts
        self.store.put('/test/skip_grabcut', True)
        self.store.put('/test/skip_planning', True)

        self.pick = PickStateMachine(store=self.store)
        self.pick.loadBoxFile('data/test/box_sizes.json')
        self.store.delete('/order')
        self.pick.loadOrderFile('data/test/order_file.json')
        self.pick.loadLocationFile('data/test/item_location_file.json')
        self.pick.loadStates()
        self.pick.setupTransitions()

    def test_runPickOrder(self):
        self.pick.runOrdered('si')
        self.assertEqual(self.pick.getAllPastEvents(), ['SI', 'FI', 'PR', 'ER', 'CI'])

    #def test_runFullOrder(self):
    #    for _ in range(10): self.pick.runOrdered('si')

    def tearDown(self):
        pass
