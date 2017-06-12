from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class CheckItem(State):

    #Confirm item was picked correctly
    def run(self):
        #assume it succeeded for now
        self.chosenItem = self.store.get('/robot/selected_item')
        self.store.put('/item/'+self.chosenItem+'/point_value', 0)
        #for i, n in self.store.get('/outcome/').items():
        #    self.store.put('/outcome/'+i, False)

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
                for i in self.notDone['contents']:
                    if i not in self.filled:
                        self.points = (20 if self.store.get('/item/'+i+'/new_item') else 10)
                        self.points += 10/(self.number-len(self.filled))
                        self.store.put('/item/'+i+'/point_value', self.points)

            #setup re-imaging of bin from which item was picked
            self.store.put('/robot/target_location', self.store.get('/robot/selected_bin'))
            self.store.put('/robot/target_bin', self.store.get('/robot/selected_bin'))
            self.store.put('/item/'+self.chosenItem+'/location', self.store.get('/robot/target_box'))


        elif alg=="stow":
            self.store.put('/item/'+self.chosenItem+'/location', self.store.get('/robot/target_bin'))
            self.store.put('/robot/target_locations', ['stow_tote'])

        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ci')
    CheckItem(myname).run()

