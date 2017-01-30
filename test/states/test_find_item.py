from unittest import TestCase, SkipTest
from states.find_item import FindItem

class FindItem_NotMoved(TestCase):
    def setUp(self):
        self.fi = FindItem('fi1')
        #default store

    def test_findStillThere(self):
        #must have selected_item defined
        self.fi.run()
        self.itemFound = self.fi.store.get('/item/'+self.fi.store.get('/robot/selected_item'))
        self.assertEqual(self.itemFound['location'], self.fi.store.get('/item/'+self.itemFound['name']+'/location'))

    def tearDown(self):
        pass
