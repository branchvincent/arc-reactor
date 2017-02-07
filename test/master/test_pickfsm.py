from test.pensive.helper import DatabaseDependentTestCase
from master.pickfsm import PickStateMachine
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

class SimplePickFSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimplePickFSM, self).setUp()
        self.pick = PickStateMachine()
        self.store = self.client.default()
        self.store.put('', initial_db)

    def test_runPickOrder(self):
        self.pick.loadStates()
        self.pick.setupTransitions()
        self.pick.runOrdered('si')
        self.assertEqual(self.pick.getAllPastEvents(), ['SI', 'FI', 'PR', 'ER'])

    def tearDown(self):
        pass
