from master.fsm import StateMachine
from states.select_item import SelectItem
from states.find_item import FindItem
from states.plan_route import PlanRoute
from states.exec_route import ExecRoute
from states.check_item import CheckItem
import json

class PickStateMachine(StateMachine):

    def loadStates(self):
        self.add('si', SelectItem('si', store=self.store))
        self.add('fi', FindItem('fi', store=self.store))
        self.add('pr', PlanRoute('pr', store=self.store))
        self.add('er', ExecRoute('er', store=self.store))
        self.add('ci', CheckItem('ci', store=self.store), endState=1)

    def setupTransitions(self):
        self.setTransition('si', 'fi', 'si', '/status/selected_item')
        self.setTransition('fi', 'pr', 'ms', '/status/selected_item_location')
        self.setTransition('pr', 'er', 'ms', '/status/route_plan')
        self.setTransition('er', 'ci', 'fi', '/status/route_exec')
        self.setTransition('ms', 'fi', 'fi', '/status/shelf_move')
        self.setTransition('ci', 'si', 'fi', '/status/item_picked')

    def loadOrderFile(self, file_location):
        with open(file_location) as data_file:
            self.order_db = json.load(data_file)
        self.store.multi_put(self.order_db)
        self.order = self.store.get('/order/').items()
        for i, n in self.order:
            for k in n['items']:
                self.points = (20 if self.store.get('/item/'+k+'/new_item') else 10)
                self.points += 10/n['number']
                self.store.put('/item/'+k+'/point_value', self.points)
                self.store.put('/item/'+k+'/order', i)

#################################################################################
def runPickFSM():
    pick = PickStateMachine()
    pick.loadStates()
    pick.setupTransitions()
    pick.loadOrderFile('test/master/order_test.json')
    pick.store.put('/status/task', None)
    for _ in range(10): pick.runOrdered('si')

if __name__ == '__main__':
    runPickFSM()
