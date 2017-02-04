from test.pensive.helper import DatabaseDependentTestCase
from test.master.test_fsm import initial_db

from states.select_item import SelectItem

class Select_First(DatabaseDependentTestCase):
    def setUp(self):
        super(Select_First, self).setUp()

        store =self.client.default()
        store.put('', initial_db)

        self.si = SelectItem('si1', store=store)
        #default store

    def test_getMaxValue(self):
        self.si.run()
        self.assertEqual(self.si.store.get('/robot/selected_item'), 'expo_markers')

    def tearDown(self):
        pass
