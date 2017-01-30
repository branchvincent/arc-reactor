from unittest import TestCase, SkipTest
from master.fsm import StateMachine
from pensive.client import PensiveClient
from states.select_item import SelectItem

class SimpleFiniteSM(TestCase):
    def setUp(self):
        self.fsm = StateMachine()
        self.store = PensiveClient().default()

    def test_SelectItem(self):
        self.fsm.add('si1', SelectItem('si1'))
        self.fsm.setCurrentState('si1')
        self.fsm.runCurrent()
        self.assertEqual(self.store.get('/robot/selected_item'), 'expo_markers')

    def tearDown(self):
        pass

