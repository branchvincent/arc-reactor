from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class CheckItem(State):
    #Confirm item was picked correctly (temp state).
    def run(self):
        #assume it succeeded for now
        self.chosenItem = self.store.get('/robot/selected_item')
        self.store.put('/item/'+self.chosenItem+'/point_value', 0)
        for i, n in self.store.get('/status/').items():
            self.store.put('/status/'+i, False)
        self.store.put('/status/item_picked', True)

        logger.info("{} item was moved successfully".format(self.chosenItem))

        alg = self.store.get('/robot/task')
        if alg=="pick":

            self.orderUp = self.store.get('/item/'+self.chosenItem+'/order')
            self.filled = self.store.get('/order/'+self.orderUp+'/filled_items')
            if self.filled == None:
                self.filled = [str(self.chosenItem)]
            else:
                self.filled.append(str(self.chosenItem))
            self.store.put('/order/'+self.orderUp+'/filled_items', self.filled)

            #update point values in the order
            self.number = self.store.get('/order/'+self.orderUp+'/number')
            if len(self.filled) < self.number:
                self.notDone = self.store.get('/order/'+self.orderUp)
                for i in self.notDone['items']:
                    if i not in self.filled:
                        self.points = (20 if self.store.get('/item/'+i+'/new_item') else 10)
                        self.points += 10/(self.number-len(self.filled))
                        self.store.put('/item/'+i+'/point_value', self.points)

if __name__ == '__main__':
    CheckItem('ci').run()

