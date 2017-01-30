from unittest import TestCase, SkipTest
from states.select_item import SelectItem

class Select_First(TestCase):
    def setUp(self):
        self.si = SelectItem('si1')
        #default store

    def test_getMaxValue(self):
        self.si.run()
        self.assertEqual(self.si.store.get('/robot/selected_item'), 'expo_markers')

    def tearDown(self):
        pass
