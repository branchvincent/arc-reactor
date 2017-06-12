from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class InspectItem(State):
    def run(self):

        #TODO check for errors from hardware

        #TODO use vector calc to look at many possible items, not just 2
        
        #check ID of item and weight from read_scales
        self.origItem = self.store.get('/robot/selected_item')
        self.newItemIDs = self.store.get('/photos/inspect/inspect_side/detections')[0] #only 1 seg?
        print "thought it was ", self.origItem
        print "now detecting the following: ", self.newItemIDs

        self.affirmID = self.newItemIDs[self.origItem]*100
            #probability that the item we thought we picked is what we have
        print "affirmID ", self.affirmID
        self.nowItem = max(self.newItemIDs, key=lambda l: self.newItemIDs[l])
        self.nowID = self.newItemIDs[self.nowItem]*100
            #probability that the item we ID'd now is correct (ish)
        print "nowID ", self.nowID

        self.readWeight = abs(self.store.get('/scales/change'))
        if self.readWeight is not None:
            # compare to origItemID weight and then nowItem weight
            self.origItemWeight = self.store.get('/item/'+self.origItem+'/mass')
            self.nowItemWeight = self.store.get('/item/'+self.nowItem+'/mass')

            # smaller value is better (closest to expect item value)
            self.origWeightError = abs(self.origItemWeight-self.readWeight)/self.origItemWeight
            self.nowWeightError = abs(self.nowItemWeight - self.readWeight)/self.nowItemWeight
            
        else: # don't consider weight a factor
            self.origWeightError = 0
            self.nowWeightError = 0

        print "origWeightError ", self.origWeightError
        print "nowWeightError ", self.nowWeightError

        if(self.origItem == self.nowItem):
                self.likelyItem = self.origItem
                print "ID'd correct item originally"
        else:
            if(self.affirmID-self.origWeightError > self.nowID-self.nowWeightError):
                self.likelyItem = self.origItem
                print "going with the first ID"
            else:
                self.likelyItem = self.nowItem
                print "going with new ID"

        self.itemisright = (self.likelyItem == self.origItem)  
        print "likelyItem is ", self.likelyItem 

        task = self.store.get('/robot/task')
        if task is None:
            self.setOutcome(False)
            raise RuntimeError("No task defined")

        elif task == 'stow':
            self.store.put('/robot/target_locations', ['binA', 'binB', 'binC'])

        elif task == 'pick':
                    

            if(self.itemisright): #proceeding to order box
                self.store.put('/robot/target_locations', [self.store.get('/robot/selected_box')])
            elif(self.store.get('/item/'+self.likelyItem+'/order') is not None): #fills an order
                self.store.put('/robot/selected_item', self.likelyItem)
                self.store.put('/robot/selected_box', self.store.get('/item/'+self.likelyItem+'/order').replace('order', 'box'))
                self.store.put('/robot/target_locations', [self.store.get('/robot/selected_box')])
            else: # go to amnesty tote
                self.store.put('/robot/target_locations', ['amnesty_tote'])

        self.setOutcome(True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ii')
    InspectItem(myname).run()

