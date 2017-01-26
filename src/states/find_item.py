from master.fsm import State

class FindItem(State):
    def run(self):
        self.ItemDict = self.store.get('/item/'+self.store.get('/robot/selected_item'))
        self.lastLoc = self.ItemDict['location']

        #etc
        #take camera pic
        #ID shelf, location, etc
        #get point cloud

        self.foundLoc = self.lastLoc #maybe

        self.store.put('/item/'+self.ItemDict['name']+'/location',  self.foundLoc)
        #etc
