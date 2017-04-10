from unittest import SkipTest

from test.pensive.helper import DatabaseDependentTestCase

from master.pickfsm import PickStateMachine
from master import workcell

class SimplePickFSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimplePickFSM, self).setUp()

        try:
            import klampt
        except ImportError:
            raise SkipTest('Klampt is not installed')

        self.store = self.client.default()
        workcell.setup_pick(
            self.store,
            location='db/item_location_file_pick.json',
            order='db/order_file.json',
            workcell='db/workcell.json'
        )

        # simulate for now
        self.store.put('/simulate/robot_motion', True)
        self.store.put('/simulate/object_detection', True)
        self.store.put('/simulate/cameras', True)

        # skip difficult to simulate parts
        self.store.put('/test/skip_grabcut', True)
        self.store.put('/test/skip_planning', True)

        self.pick = PickStateMachine(store=self.store)
        self.pick.loadStates()
        self.pick.setupTransitions()

    def test_runPickOrder(self):
        pass
        #self.pick.runOrdered('si')
        #self.assertEqual(self.pick.getAllPastEvents(), ['SI', 'FI', 'PR', 'ER', 'CI'])

    #def test_runFullOrder(self):
    #    for _ in range(10): self.pick.runOrdered('si')

    def tearDown(self):
        pass
