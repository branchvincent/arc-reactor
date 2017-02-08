from test.pensive.helper import DatabaseDependentTestCase
from test.master.test_fsm import initial_db

from states.select_item import SelectItem

class Select_First(DatabaseDependentTestCase):
    def setUp(self):
        super(Select_First, self).setUp()
        self.store = self.client.default()
        self.store.put('', initial_db)
        self.si = SelectItem('si', store=self.store)

    def test_getMaxValue(self):
        self.store.put('/status/task', None)
        self.si.run()
        self.assertEqual(self.si.store.get('/robot/selected_item'), 'kong_duck_dog_toy')

    def tearDown(self):
        pass
