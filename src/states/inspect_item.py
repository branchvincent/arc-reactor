from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class InspectItem(State):
    """
    Input:
        - /robot/selected_item: selected/chosen Item we think we have
        - /photos/inspect/inspect_side/detections: likely IDs of held item
        - /scales/change: change in weight after item picked up
        - /item/[item name]/mass: masses of considered items
        - /robot/task: pick, stow, final
    (pick):
        - /item/[item name]/order: if item ordered, which box
        - /order/[order name]/filled_items
        - /order/[order name]/number
    (stow):
        -
    Output:
        - /failure/inspect_item: failure string
    (pick):
        - /robot/target_locations: if ordered, box ID
        - /robot/target_box: if ordered, box ID
        - /robot/selected_box: where we put the item
    (stow):
        - /robot/target_location: (HACK) binA, binB, binC
    Failure Cases:
        - RuntimeError if no task defined
    Dependencies:
        - item at the inspection station
    """
    def run(self):
        #TODO check for errors from hardware

        #TODO use vector calc to look at many possible items, not just 2

        #check ID of item and weight from read_scales
        self.origItem = self.store.get('/robot/selected_item')
        if self.store.get('/photos/inspect/inspect_side/detections'):
            self.newItemIDs = self.store.get('/photos/inspect/inspect_side/detections')[0] #only 1 seg?
        else:
            logger.error('detections failed at inspection station -> defaulting to original')
            self.newItemIDs = self.origItem
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
            print "origWeight ", self.origItemWeight
            self.nowItemWeight = self.store.get('/item/'+self.nowItem+'/mass')
            print "nowItemWeight ", self.nowItemWeight

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

        elif(self.readWeight<0.005):
            self.setOutcome(False)
            self.store.put(['failure', self.getFullName()], "NoItemError")
            logger.error('Likely nothing was picked up: no weight change detected.')

        elif task == 'stow':
            self.store.put('/robot/target_locations', ['binA', 'binB', 'binC'])
            self._mark_grasp_succeeded()

        elif task == 'pick':

            if(self.itemisright and self.store.get('/item/'+self.likelyItem+'/order') is not None): #proceeding to order box
                self.store.put('/robot/selected_box', self.store.get('/item/'+self.likelyItem+'/order').replace('order', 'box'))
                self.store.put('/robot/target_locations', [self.store.get('/robot/selected_box')])
                self.store.put('/robot/target_box', self.store.get('/robot/selected_box'))
                self.setOutcome(True)
            elif(self.store.get('/item/'+self.likelyItem+'/order') is not None): #fills an order
                self.store.put('/robot/selected_item', self.likelyItem)
                self.store.put('/robot/selected_box', self.store.get('/item/'+self.likelyItem+'/order').replace('order', 'box'))
                self.store.put('/robot/target_locations', [self.store.get('/robot/selected_box')])
                self.setOutcome(True)
            else: # go to amnesty tote
                self.store.put('/robot/target_locations', ['box1B2'])
                self.store.put('/robot/target_box', 'box1B2')
                self.store.put('/robot/selected_box', 'box1B2')
        #                self.setOutcome(False)
                self.setOutcome(True)

            self._mark_grasp_succeeded()

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None):
            return 0
            #no failure detected, no suggestions!
        elif(self.whyFail == "NoItemError"):
            return 1
            #go to first fallback state
        else:
            return 0
            #again, no suggestions!

    def _mark_grasp_succeeded(self):
        failed_grasps = self.store.get(['robot', 'failed_grasps'], [])
        target_grasp = self.store.get(['robot', 'target_grasp'])

        logger.info('grasp succeeded at {}'.format(target_grasp['center']))

        tolerance = self.store.get('/planner/grasp_success_radius', 0.1)
        i = 0
        while i < len(failed_grasps):
            grasp = failed_grasps[i]

            distance = ((grasp['center'] - target_grasp['center'])**2).sum()**0.5
            if distance < tolerance:
                logger.info('grasp cleared at {}'.format(grasp['center']))
                del failed_grasps[i]
            else:
                i += 1

        self.store.put(['robot', 'failed_grasps'], failed_grasps)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ii')
    InspectItem(myname).run()
