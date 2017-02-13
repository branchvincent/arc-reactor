from unittest import SkipTest

from test.pensive.helper import DatabaseDependentTestCase

from master.pickfsm import PickStateMachine

import json
with open('test/master/initial_db.json') as data_file:
    initial_db = json.load(data_file)

class SimplePickFSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimplePickFSM, self).setUp()

        try:
            import klampt
        except ImportError:
            raise SkipTest('Klampt is not installed')

        self.store = self.client.default()
        self.store.put('', initial_db)
        self.store.put('/status/task', 'pick')
        #simulate for now for robot controller
        self.store.put('/simulate/robot_motion', True)

        self.pick = PickStateMachine(store=self.store)
        self.pick.loadOrderFile('test/master/order_test.json')
        self.pick.loadStates()
        self.pick.setupTransitions()

    def test_runPickOrder(self):
        self.pick.runOrdered('si')
        self.assertEqual(self.pick.getAllPastEvents(), ['SI', 'FI', 'PR', 'ER', 'CI'])

    def test_runFullOrder(self):
        for _ in range(10): self.pick.runOrdered('si')

    def tearDown(self):
        pass
