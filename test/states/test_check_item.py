from test.pensive.helper import DatabaseDependentTestCase
from test.master.test_fsm import initial_db

from states.check_item import CheckItem

class CheckItemPick(DatabaseDependentTestCase):
    def setUp(self):
        super(CheckItemPick, self).setUp()
        self.store = self.client.default()
        self.store.put('', initial_db)
        self.ci = CheckItem('ci', store=self.store)

    def test_checkGotItem(self):
        pass

    def tearDown(self):
        pass
