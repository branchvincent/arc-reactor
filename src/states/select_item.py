from master.fsm import State

class SelectItem(State):
    def run(self):
        self.alg = self.store.get('/status/task')
        self.itemList = self.store.get('/item/')
        if self.alg == None:
            self.chosenItem = max(self.itemList, key=lambda l: self.itemList[l]['point_value'])

        elif self.alg == 'pick':
            self.chosenItem = max(self.itemList, key=lambda l: self.itemList[l]['point_value'])

        elif self.alg == 'stow':
            #self.points = 
            pass

        elif self.alg == 'final':
            #self.points = 
            pass
            
        else:
            raise RuntimeError("Algorithm for selection is None, pick, stow, or final")
        
        self.store.put('/robot/selected_item', self.chosenItem)
        print "chosen item: ", self.chosenItem, " worth ", self.store.get('/item/'+self.chosenItem+'/point_value')
        self.store.put('/status/selected_item', self.chosenItem is not None)
