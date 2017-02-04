from test.pensive.helper import DatabaseDependentTestCase

from states.move_shelf import MoveShelf

class MoveShelf_Direct(DatabaseDependentTestCase):
    def setUp(self):
        super(MoveShelf_Direct, self).setUp()

        self.ms = MoveShelf('ms1', store=self.client.default())
        self.ms.store.put('/shelf/current_angle', '0')
        self.ms.store.put('/shelf/goal_angle', '90')
        #default store

    def test_goDirect(self):
        self.ms.run()
        self.assertEqual(self.ms.store.get('/shelf/current_angle'), self.ms.store.get('/shelf/goal_angle'))

    def tearDown(self):
        pass

