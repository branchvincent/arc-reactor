from test.pensive.helper import DatabaseDependentTestCase
from test.master.test_fsm import initial_db

from states.find_item import FindItem
class FindItem_NotMoved(DatabaseDependentTestCase):
    def setUp(self):
        super(FindItem_NotMoved, self).setUp()
        store = self.client.default()

        store.put('', initial_db)
        store.put('/robot/selected_item', 'expo_markers')

        self.fi = FindItem('fi1', store=store)
        #default store

    def test_findStillThere(self):
        #must have selected_item defined
        self.fi.run()
        self.itemFound = self.fi.store.get('/item/'+self.fi.store.get('/robot/selected_item'))
        self.assertEqual(self.itemFound['location'], self.fi.store.get('/item/'+self.itemFound['name']+'/location'))

    def tearDown(self):
        pass
