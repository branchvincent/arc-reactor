
from test.pensive.helper import DatabaseDependentTestCase

from master.fsm import StateMachine

from states.select_item import SelectItem
from states.find_item import FindItem

import json

class SimpleFiniteSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimpleFiniteSM, self).setUp()

        self.fsm = StateMachine()
        self.store = self.client.default()

        self.store.put('', json.loads('''
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
'''))

    def test_SelectItem(self):
        self.fsm.add('si1', SelectItem('si1', store=self.store))
        self.fsm.setCurrentState('si1')
        self.fsm.runCurrent()
        self.assertEqual(self.store.get('/robot/selected_item'), 'expo_markers')

    def test_runAll(self):
        self.fsm.add('si1', SelectItem('si1', store=self.store))
        self.fsm.add('fi1', FindItem('fi1', store=self.store))
        self.fsm.runAll()
        self.assertTrue(True)

    def tearDown(self):
        pass
