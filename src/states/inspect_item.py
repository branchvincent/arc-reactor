import logging

import numpy

from master.fsm import State

logger = logging.getLogger(__name__)

class InconsistentItemsError(RuntimeError):
    pass

class MissingDetectionsError(RuntimeError):
    pass

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

        # NOTE: recommended usage of multiple detections
        # try:
        #     self.newItemIDs = self._combine_multi_detections()
        #     self.newItemIDs = self._filter_by_mass_absolute_error(self.newItemIDs)
        # except (InconsistentItemsError, MissingDetectionsError):
        #     logger.exception('detections failed at inspection station -> defaulting to original')
        #     self.newItemIDs = {self.origItem: 1}

        if self.store.get('/photos/inspect/inspect_side/detections'):
            self.newItemIDs = self.store.get('/photos/inspect/inspect_side/detections')[0] #only 1 seg?
        else:
            logger.error('detections failed at inspection station -> defaulting to original')
            self.newItemIDs = {self.origItem: 1}
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
            self.setOutcome(True)
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
                self.store.put('/robot/selected_item', self.likelyItem)
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

    def _combine_multi_detections(self):
        '''
        Average the results of multiple detections using the pixel count for the
        detected segment as a weight.
        '''

        grasp = self.store.get('/robot/target_grasp')
        grasp_photo_url = ['photos', grasp['location'], grasp['camera']]

        side_photo_url = ['photos', 'inspect', 'inspect_side']
        below_photo_url = ['photos', 'inspect', 'inspect_below']

        # all pixels in an inspect photo belong to segment 1
        inspect_segment = 1

        photos_segments = [
            (grasp_photo_url, grasp['segment_id']),
            (side_photo_url, inspect_segment),
            (below_photo_url, inspect_segment)
        ]

        # combine all the detections using pixel count as a weight
        multi_detections = {}
        for (i, (url, segment)) in enumerate(photos_segments):
            logger.debug('using photo {} with segment {}'.format(url, segment))

            # retrieve the detections for this segment
            try:
                detections = self.store.get(url + ['detections'], [])[segment - 1]
            except (KeyError, IndexError):
                raise MissingDetectionsError()

            # compute the segment pixel count
            labeled_image = self.store.get(url + ['labeled_image'])
            pixel_count = numpy.count_nonzero(labeled_image == segment)

            for (name, confidence) in detections.items():
                # look up the prior result
                try:
                    prior = multi_detections[name]
                except KeyError:
                    # only allow missing names for first URL
                    if i == 0:
                        prior = 0
                    else:
                        raise InconsistentItemsError('detections have different items: {}'.format(name))

                # perform the running sum
                multi_detections[name] = prior + pixel_count * confidence

        # normalize the multidetections
        total = sum(multi_detections.values())
        multi_detections = {k: v / total for (k, v) in multi_detections.items()}

        return multi_detections

    def _filter_by_mass_absolute_error(self, detections, threshold=0.005):
        '''
        Zero all detections where the mass absolute error exceeds the threshold.

        The EasyWeigh PX-60-PL scales have a 5 g interval.
        '''

        # read the mass change
        measured_mass = abs(self.store.get(['scales', 'change']))

        items = self.store.get(['item'])
        for name in detections:
            if name not in items:
                # skip items not in the database
                # NOTE: this is needed because detections includes all items, not just those in the workcell
                continue

            if abs(items[name]['mass'] - measured_mass) > threshold:
                # eliminate this detection
                detections[name] = 0
                logger.debug('rejected "{}" by mass error: {:.3f} kg'.format(name, items[name]['mass'] - measured_mass))

        return detections

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ii')
    InspectItem(myname).run()
