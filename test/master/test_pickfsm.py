from unittest import SkipTest

import numpy

from test.pensive.helper import DatabaseDependentTestCase

from master.pickfsm import PickStateMachine

import json
with open('test/master/test_022717_badik2.json') as data_file:
    initial_db = json.load(data_file)

class SimplePickFSM(DatabaseDependentTestCase):
    def setUp(self):
        super(SimplePickFSM, self).setUp()

        try:
            import klampt
        except ImportError:
            raise SkipTest('Klampt is not installed')

        self.store = self.client.default()
        self.store.put('', initial_db)
        self.store.put('/status/task', 'pick')
        #simulate for now
        self.store.put('/simulate/robot_motion', True)
        self.store.put('/simulate/object_detection', True)
        self.store.put('/simulate/cameras', True)

        self.store.put('/simulate/headless', True)
        self.store.put('/camera/shelf0/pose', numpy.array(
            [[ 0.70711,  0.2706 , -0.65328,  1.58   ],
             [ 0.70711, -0.2706 ,  0.65328, -0.47   ],
             [    -0.     , -0.92388, -0.38268,  1.04   ],
             [ 0.     ,  0.     ,  0.     ,  1.     ]]
        ))

        self.pick = PickStateMachine(store=self.store)
        self.pick.loadOrderFile('test/master/order_test.json')
        self.pick.loadStates()
        self.pick.setupTransitions()

    def test_runPickOrder(self):
        self.pick.runOrdered('si')
        self.assertEqual(self.pick.getAllPastEvents(), ['SI', 'FI', 'PR', 'ER', 'CI'])

    #def test_runFullOrder(self):
    #    for _ in range(10): self.pick.runOrdered('si')

    def tearDown(self):
        pass
