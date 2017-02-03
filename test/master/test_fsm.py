from unittest import TestCase, SkipTest
from master.fsm import StateMachine
from pensive.client import PensiveClient
from pensive.server import PensiveServer
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute

class SimpleFiniteSM(TestCase):
    def setUp(self):
        self.fsm = StateMachine()
        self.store = PensiveClient().default()

    def test_SelectItem(self):
        self.fsm.add('si1', SelectItem('si1'))
        self.fsm.setCurrentState('si1')
        self.fsm.runCurrent()
        self.assertEqual(self.store.get('/robot/selected_item'), 'expo_markers')

    def test_runNotAll(self):
        self.fsm.add('si1', SelectItem('si1'))
        self.fsm.add('fi1', FindItem('fi1'), endState=1)
        self.fsm.add('pr1', PlanRoute('pr1'))
        self.fsm.runAll()
        self.assertTrue(self.fsm.getCurrentState(), "FI1")

    def test_runOrder(self):
        self.fsm.add('si1', SelectItem('si1'))
        self.fsm.add('fi1', FindItem('fi1'), endState=1)
        self.fsm.add('pr1', PlanRoute('pr1'))
        self.fsm.runAll()
        self.assertEqual(self.fsm.getAllPastEvents(), ['SI1', 'FI1'])

    def tearDown(self):
        pass

