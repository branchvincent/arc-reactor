# pylint: disable=line-too-long,missing-docstring,invalid-name,protected-access

from test.pensive.helper import DatabaseDependentTestCase

from hardware.vacuum.rpi import Vacuum, ConnectionError

class Skip(object):
    class RaspberryPiVacuumTest(DatabaseDependentTestCase):
        def setUp(self):
            super(Skip.RaspberryPiVacuumTest, self).setUp()
            self.store = self.client.default()

        def test_on(self):
            vacuum = Vacuum(store=self.store)
            vacuum.on()

            self.assertTrue(self.store.get('/vacuum/status'))
            self.assertTrue(vacuum.is_on())
            self.assertFalse(vacuum.is_off())

        def test_off(self):
            vacuum = Vacuum(store=self.store)
            vacuum.off()

            self.assertFalse(self.store.get('/vacuum/status'))
            self.assertFalse(vacuum.is_on())
            self.assertTrue(vacuum.is_off())

        # def test_auto_off(self):
        #     Vacuum(store=self.store).on()
        #     vacuum.on()
        #     del vacuum

        #     self.assertFalse(self.store.get('/vacuum/status'))
        #     self.assertFalse(vacuum.is_on())
        #     self.assertTrue(vacuum.is_off())

        def tearDown(self):
            pass

class RaspberryPiVacuumTest_Simulate(Skip.RaspberryPiVacuumTest):
    def setUp(self):
        super(RaspberryPiVacuumTest_Simulate, self).setUp()
        self.store.put('/simulate/vacuum', True)

class RaspberryPiVacuumTest_Real(Skip.RaspberryPiVacuumTest):
    def setUp(self):
        super(RaspberryPiVacuumTest_Real, self).setUp()
        self.store.put('/simulate/vacuum', False)

        try:
            Vacuum(store=self.store)
        except ConnectionError:
            self.skipTest('no vacuum connected')
