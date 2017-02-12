from master.fsm import State
from simulation.grabcut import GrabObject

class FindItem(State):
    def run(self):
        self.ItemDict = self.store.get('/item/'+self.store.get('/robot/selected_item'))
        self.lastLoc = self.ItemDict['location']

        if self.store.get('/simulate/object_detection'):
            self.grab = GrabObject()
            self.grab.run()
            #now need to get mask for point cloud
            self.mask = self.store.get('/camera/camera1/mask')
        else:
            # for all cameras available, do:
            pass

        self.foundLoc = self.lastLoc #maybe
        
        self.store.put('/item/'+self.ItemDict['name']+'/location',  self.foundLoc)
        self.store.put('/status/selected_item_location', True)
        #etc

