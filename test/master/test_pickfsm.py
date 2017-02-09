from test.pensive.helper import DatabaseDependentTestCase
from master.pickfsm import PickStateMachine
from pensive.client import PensiveClient
from pensive.server import PensiveServer
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute

import json
with open('test/master/initial_db.json') as data_file:
    initial_db = json.load(data_file)

class SimplePickFSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimplePickFSM, self).setUp()
        self.store = self.client.default()
        self.store.put('', initial_db)
        self.store.put('/status/task', 'pick')

        self.pick = PickStateMachine(store=self.store)
        self.pick.loadOrderFile('test/master/order_test.json')

    def test_runPickOrder(self):
        self.pick.loadStates()
        self.pick.setupTransitions()
        self.pick.runOrdered('si')
        self.assertEqual(self.pick.getAllPastEvents(), ['SI', 'FI', 'PR', 'ER', 'CI'])

    def tearDown(self):
        pass
