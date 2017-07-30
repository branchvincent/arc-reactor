from master.fsm import State
import logging; logger = logging.getLogger(__name__)
from util import dump_location

class CheckItem(State):
    """
    Input:
        - /robot/selected_item: selected/chosen Item
    (pick):
        - /item/[item name]/order: if item ordered, which box
        - /order/[order name]/filled_items
        - /order/[order name]/number
    (stow):
        -
    Output:
        - /item/[item name]/location: if found, current item location
        - /robot/target_locations: array of target locations for next states (HACK?)
        - /failure/check_item: failure string
    (pick):
        - /order/[order name]/filled_items: updated list
        - /item/[all items]/point_value: updated point value for all items
        - /robot/target_location: (HACK)
        - /robot/target_bin: value from item/[]/selected_bin. use past knowledge.
    Failure Cases:
        - [connection to db?]
    Dependencies:
        - something was placed somewhere
    """
    #Confirm item was picked correctly
    def run(self):
        #assume it succeeded for now
        self.chosenItem = self.store.get('/robot/selected_item')
        if(self.chosenItem=='unknown'):
            logger.info("Unknown item moved to bin B")

        else:
            #TODO get outcome from location?
            self.store.put('/item/'+self.chosenItem+'/point_value', 0)
            logger.info("{} item was moved successfully".format(self.chosenItem))

        alg = self.store.get('/robot/task')
        if alg=="pick":
            if(self.store.get('/item/'+self.chosenItem+'/order/') is not None):
                self.orderUp = self.store.get('/item/'+self.chosenItem+'/order')
                self.filled = self.store.get('/order/'+self.orderUp+'/filled_items')
                if self.filled == None:
                    self.filled = [str(self.chosenItem)]
                else:
                    self.filled.append(str(self.chosenItem))
                self.store.put('/order/'+self.orderUp+'/filled_items', self.filled)

                #TODO remove item from needed_items

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
                self.store.put('/robot/target_view_location', self.store.get('/robot/selected_bin'))
                self.store.put('/robot/target_locations', ["binA", "binB", "binC"])
                self.store.put('/robot/target_bin', self.store.get('/robot/selected_bin'))
                #updated location file and store with new location of item
                self.store.put('/item/'+self.chosenItem+'/location', self.store.get('/robot/target_box'))
                dump_location.run(self.store)
            else: #dropped in bin A
                self.store.put('/robot/target_view_location', self.store.get('/robot/selected_bin'))
                self.store.put('/robot/target_locations', ["binA", "binB", "binC"])
                self.store.put('/robot/target_bin', self.store.get('/robot/selected_bin'))
                self.store.put('/item/'+self.chosenItem+'/location', 'binA')
                dump_location.run(self.store)

        elif alg=="stow":
            if(self.chosenItem!='unknown'):
                self.store.put('/item/'+self.chosenItem+'/location', self.store.get('/robot/target_bin'))
                self.store.put('/robot/target_view_location', self.store.get('/robot/target_bin'))
            self.store.put('/robot/target_locations', ['stow_tote'])
            dump_location.run(self.store)
        self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ci')
    CheckItem(myname).run()

