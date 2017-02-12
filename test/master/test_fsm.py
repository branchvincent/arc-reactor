
from test.pensive.helper import DatabaseDependentTestCase

from master.fsm import StateMachine

from pensive.client import PensiveClient
from pensive.server import PensiveServer

from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute

import json
with open('test/master/initial_db.json') as data_file:
    initial_db = json.load(data_file)

class SimpleFiniteSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimpleFiniteSM, self).setUp()
        self.store = self.client.default()
        self.store.put('', initial_db)
        self.store.put('/robot/task', None)

        self.fsm = StateMachine(store=self.store)

    def test_SelectItem(self):
        self.fsm.add('si1', SelectItem('si1', store=self.store))
        self.fsm.setCurrentState('si1')
        self.fsm.runCurrent()
        self.assertEqual(self.store.get('/robot/selected_item'), 'kong_duck_dog_toy')

    def test_runNotAll(self):
        self.fsm.add('si1', SelectItem('si1', store=self.store))
        self.fsm.add('fi1', FindItem('fi1', store=self.store), endState=1)
        self.fsm.add('pr1', PlanRoute('pr1', store=self.store))
        self.fsm.runAll()
        self.assertTrue(self.fsm.getCurrentState(), "FI1")

    def test_runOrder(self):
        self.fsm.add('si1', SelectItem('si1', store=self.store))
        self.fsm.add('fi1', FindItem('fi1', store=self.store), endState=1)
        self.fsm.add('pr1', PlanRoute('pr1', store=self.store))
        self.fsm.runAll()
        self.assertEqual(self.fsm.getAllPastEvents(), ['SI1', 'FI1'])

    def tearDown(self):
        pass
