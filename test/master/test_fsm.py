
from test.pensive.helper import DatabaseDependentTestCase

from master.fsm import StateMachine

from states.select_item import SelectItem
from states.find_item import FindItem

class SimpleFiniteSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimpleFiniteSM, self).setUp()

        self.fsm = StateMachine()
        self.store = self.client.default()

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
