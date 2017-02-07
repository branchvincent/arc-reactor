from master.fsm import State

class SelectItem(State):
    def run(self):
        #pick item from db
        self.itemList = self.store.get('/item/').keys()
        self.maxItem = max(self.itemList, key=lambda l: self.store.get('/item/'+l+'point_value'))
        
        self.chosenItem = self.maxItem
        
        self.store.put('/robot/selected_item', self.chosenItem)

        self.store.put('/status/selected_item', self.chosenItem is not None)
     
    def whatlog(self):
        #record somehow
        print("Selected item ", self.chosenItem)
