
from test.pensive.helper import DatabaseDependentTestCase

from master.fsm import StateMachine

from pensive.client import PensiveClient
from pensive.server import PensiveServer

from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute

import json
initial_db = json.loads('''
{
    "robot": {
        "selected_item": null
    },

    "item": {
        "dr_dobbs_bottle_brush": {
            "name": "dr_dobbs_bottle_brush",
            "display_name": "Dr. Dobbs Bottle Brush",
            "point_value": 10,
            "location": "binA"
        },
        "kong_duck_dog_toy": {
            "name": "kong_duck_dog_toy",
            "display_name": "Kong Duck Dog Toy",
            "point_value": 20,
            "location": "binA"
        },
        "expo_markers": {
            "name": "expo_markers",
            "display_name": "Expo Markers",
            "point_value": 20,
            "location": "binB"
        }
    }

}
''')

class SimpleFiniteSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimpleFiniteSM, self).setUp()

        self.fsm = StateMachine()
        self.store = self.client.default()

        self.store.put('', initial_db)

    def test_SelectItem(self):
        self.fsm.add('si1', SelectItem('si1', store=self.store))
        self.fsm.setCurrentState('si1')
        self.fsm.runCurrent()
        self.assertEqual(self.store.get('/robot/selected_item'), 'expo_markers')

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
