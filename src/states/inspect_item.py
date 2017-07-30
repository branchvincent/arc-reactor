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
        self.likelyItem = self.store.get('/robot/selected_item')

        # NOTE: recommended usage of multiple detections
        try:
            self.newItemIDs = self._combine_multi_detections()
            self.store.put('/debug/detections_combo', self.newItemIDs)
            if(max(self.newItemIDs.values())<0.9):
                self.newItemIDs = self._filter_by_mass_absolute_error(self.newItemIDs)
                self.store.put('/debug/detections_mass', self.newItemIDs)
        except (InconsistentItemsError, MissingDetectionsError):
            logger.exception('detections failed at inspection station -> defaulting to original')
            self.newItemIDs = {self.origItem: 1}

        # if self.store.get('/photos/inspect/inspect_side/detections'):
        #     self.newItemIDs = self.store.get('/photos/inspect/inspect_side/detections')[0] #only 1 seg?
        # else:
        #     logger.error('detections failed at inspection station -> defaulting to original')
        #     self.newItemIDs = {self.origItem: 1}
        print "thought it was ", self.origItem
        print "now detecting the following: ", self.newItemIDs

        self.affirmID = self.newItemIDs[self.origItem]*100
            #probability that the item we thought we picked is what we have
        print "affirmID ", self.affirmID
        self.nowItem = max(self.newItemIDs, key=lambda l: self.newItemIDs[l])
        print "nowItem", self.nowItem
        self.nowID = self.newItemIDs[self.nowItem]*100
            #probability that the item we ID'd now is correct (ish)
        print "nowID ", self.nowID

        self.readWeight = abs(self.store.get('/scales/change'))

        if(self.nowID>0.9):
            logger.info("Really good detection")
            self.likelyItem = self.nowItem
            self.itemisright = True

        elif(self.nowID>=1e-9):
            logger.info("At least some detections were generated")
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
                logger.warning("Scales were unable to be read")
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
            if(self.nowID==0):
                logger.warning("Weight change found but no detections")
                try:
                    self.weightItem = self._detect_by_mass_only(self.newItemIDs)
                except RuntimeError as e:
                    pass
                if self.weightItem is not None:
                    self.store.put('/robot/selected_item', self.weightItem)
                    self.store.put('/robot/target_locations', ['binA', 'binB', 'binC'])
                    self.setOutcome(True)
                else:
                    self.store.put('/robot/target_locations', ['amnesty_tote'])
                #self.store.put('/robot/target_locations', ['binB'])
                    self.store.put('/robot/selected_item', 'unknown')
                    self.store.put(['failure', self.getFullName()], "NoDetections")
                #self.store.put(['failure', self.getFullName()], "NoItemError")
                    self.setOutcome(False)
            else:
                self.store.put('/robot/target_locations', ['binA', 'binB', 'binC'])
                self.store.put('/robot/selected_item', self.likelyItem)
                self._mark_grasp_succeeded()
                self.setOutcome(True)

        elif task == 'pick':
            if(self.nowID==0):
                logger.warning("Weight change found but no detections")
                self.store.put(['failure', self.getFullName()], "NoDetections")
                self.setOutcome(False)
            elif(self.itemisright and self.store.get('/item/'+self.likelyItem+'/order') is not None): #proceeding to order box
                self.store.put('/robot/selected_box', self.store.get('/item/'+self.likelyItem+'/order').replace('order', 'box'))
                self.store.put('/robot/target_locations', [self.store.get('/robot/selected_box')])
                self.store.put('/robot/target_box', self.store.get('/robot/selected_box'))
                self.setOutcome(True)
                self._mark_grasp_succeeded()
            elif(self.store.get('/item/'+self.likelyItem+'/order') is not None): #fills an order
                self.store.put('/robot/selected_item', self.likelyItem)
                self.store.put('/robot/selected_box', self.store.get('/item/'+self.likelyItem+'/order').replace('order', 'box'))
                self.store.put('/robot/target_locations', [self.store.get('/robot/selected_box')])
                self.setOutcome(True)
                self._mark_grasp_succeeded()
            else: #put back
                logger.warning("Item ID'd but not ordered")
                #self.replaced = self.store.get('/item/'+self.likelyItem+'/replaced', 0)
                #if self.replaced==0:
                    #first time replacing, so put back in exact spot
                #    self.store.put('/item/'+self.likelyItem+'/replaced', 1)
                #    self.store.put(['failure', self.getFullName()], "WrongItem")
                #else: #already replaced, move to other bin
                    #self.store.put('/item/'+self.likelyItem+'/replaced', 0)
                    #self.store.put('/robot/target_locations', [x for x in ['binA', 'binB', 'binC'] if x!=self.store.get('/item/'+self.likelyItem+'/location')])
                self.store.put('/robot/target_locations', ['binA'])
                self.store.put('/item/'+self.likelyItem+'/location', 'binA')
                self.store.put(['failure', self.getFullName()], "WrongItemMove")
                self.setOutcome(False)


    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None):
            return 0
            #no failure detected, no suggestions!
        elif(self.whyFail == "NoItemError"):
            return 1
            #go to first fallback state
        elif(self.whyFail == "NoDetections" or self.whyFail == "WrongItem" or self.whyFail == "UnknownItem" or self.whyFail=="MissingDetectionsError"):
            #pick only, put item back; stow put in amnesty
            return 2
        elif(self.whyFail == "WrongItemMove"):
            return 3
        else:
            return 0
            #again, no suggestions!

    def _mark_grasp_succeeded(self):
        failed_grasps = self.store.get(['robot', 'failed_grasps'], [])
        target_grasp = self.store.get(['robot', 'target_grasp'])

        logger.info('successful grasp in {location} at {center}'.format(**target_grasp))


        tolerance = self.store.get('/planner/grasp_success_radius', 0.1)

        remembered_grasps = []
        for grasp in failed_grasps:
            distance = ((grasp['center'] - target_grasp['center'])**2).sum()**0.5

            if grasp['location'] == target_grasp['location'] and distance < tolerance:
                logger.info('cleared grasp in {location} at {center} (age {age})'.format(**grasp))
            else:
                remembered_grasps.append(grasp)

        self.store.put(['robot', 'failed_grasps'], remembered_grasps)

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
        first = True

        for (i, (url, segment)) in enumerate(photos_segments):
            logger.debug('using photo {} with segment {}'.format(url, segment))

            # retrieve the detections for this segment
            try:
                detections = self.store.get(url + ['detections'], [])[segment - 1]
            except (KeyError, IndexError):
                logger.warn('no detections for "{}"'.format(url))
                continue

            # compute the segment pixel count
            labeled_image = self.store.get(url + ['labeled_image'])
            pixel_count = numpy.count_nonzero(labeled_image == segment)

            for (name, confidence) in detections.items():
                # look up the prior result
                try:
                    prior = multi_detections[name]
                except KeyError:
                    # only allow missing names for first URL
                    if first:
                        prior = 0
                    else:
                        raise InconsistentItemsError('detections have different items: {}'.format(name))

                # perform the running sum
                multi_detections[name] = prior + pixel_count * confidence

            first = False

        if not multi_detections:
            raise MissingDetectionsError()

        # normalize the multidetections
        total = sum(multi_detections.values())
        multi_detections = {k: v / total for (k, v) in multi_detections.items()}

        return multi_detections

    def _filter_by_mass_absolute_error(self, detections, threshold=0.015):
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

    def _detect_by_mass_only(self, detections):
        '''
        If no detections, try selecting by weight only.
        '''
        logger.debug("Trying to select by weight only")
        # read the mass change
        measured_mass = abs(self.store.get(['scales', 'change']))
        weight_error = {}

        items = self.store.get(['item'])
        for name in detections:
            if name not in items:
                # skip items not in the database
                # NOTE: this is needed because detections includes all items, not just those in the workcell
                continue

            weight_error[name] = abs(items[name]['mass'] - measured_mass)

        if(len(weight_error)>=2):
            self.sortedWeights = weight_error.items()
            self.sortedWeights.sort(key=lambda l: l[1])

            if((self.sortedWeights[0][1] - self.sortedWeights[1][1])>0.010 and self.sortedWeights[0][1]<=0.005):
                return self.sortedWeights[0][0]
                self.store.put('/debug/weight_error', self.sortedWeights[0][0])
            else:
                return None
        else:
            return None

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ii')
    InspectItem(myname).run()
