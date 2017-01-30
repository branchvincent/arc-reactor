from unittest import TestCase, SkipTest
from master.fsm import StateMachine
from pensive.client import PensiveClient
from states.select_item import SelectItem
from states.find_item import FindItem

class SimpleFiniteSM(TestCase):
    def setUp(self):
        self.fsm = StateMachine()
        self.store = PensiveClient().default()

    def test_SelectItem(self):
        self.fsm.add('si1', SelectItem('si1'))
        self.fsm.setCurrentState('si1')
        self.fsm.runCurrent()
        self.assertEqual(self.store.get('/robot/selected_item'), 'expo_markers')

    def test_runAll(self):
        self.fsm.add('si1', SelectItem('si1'))
        self.fsm.add('fi1', FindItem('fi1'))
        self.fsm.runAll()
        self.assertTrue(True)

    def tearDown(self):
        pass

