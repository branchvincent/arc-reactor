from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class SelectItem(State):
    """
    Input:
        - /robot/task: pick, stow, final
        - /item/: list of all items
        - /robot/failed_grasps: list of old grasps or []
        - [list of photo urls]/vacuum_grasps: grasps ID'd in that photo
        - [list of photo urls]/detections: likely item IDs in the segment
    (stow):
        - /robot/target_photo_url: to get list of photo urls
    Output:
        - /robot/target_grasp: grasps selected to grab item
        - /robot/grasp_location: location (eg binA) where grasp is selected
        - /robot/failed_grasps: updated list of old grasps
        - /robot/selected_item: item we think we're picking up
        - /failure/select_item: failure string
    (pick):
        - /robot/target_box: where to put item eventually
        - /robot/selected_bin: value from item/[]/selected_bin. use past knowledge.
    Failure Cases:
        - RuntimeError if /robot/task is incorrect
    Dependencies:
        - EvalGrasp was run
    """
    def run(self):
        self.alg = self.store.get('/robot/task')
        self.itemList = self.store.get('/item/')
        logger.info("Algorithm is {}".format(self.alg))

        failed_grasps = self.store.get('/robot/failed_grasps', [])

        if self.alg == None:
            self.chosenItem = max(self.itemList, key=lambda l: self.itemList[l]['point_value'])

        elif self.alg == 'pick':
            self.maxGrasp = None

            urlA = ['photos', 'binA', 'tcp']
            self.graspsA = self.store.get(urlA + ['vacuum_grasps'])
            urlB = ['photos', 'binB', 'tcp']
            self.graspsB = self.store.get(urlB + ['vacuum_grasps'])
            urlC = ['photos', 'binC', 'tcp']
            self.graspsC = self.store.get(urlC + ['vacuum_grasps'])

            #self.graspBins = self.graspsA + self.graspsB + self.graspsC
            self.graspBins = self.graspsB + self.graspsC

            # forget old grasps
            self._expire_grasp_attempts('binA')
            self._expire_grasp_attempts('binB')
            self._expire_grasp_attempts('binC')

            # reject repeat grasps
            self.grasps = [g for g in self.graspBins if not self._check_failed_grasp(failed_grasps, g)]
            if not self.grasps:
                raise RuntimeError('all grasps were rejected')

            self.grasps.sort(key=lambda l: -l['score'])

            #self.chosenGrasps = [l for l in self.grasps if self.store.get('/item/'+self._most_likely_item(l)+'/order') is not None]
            self.chosenGrasps = self._screen_chosen_items(self.grasps)

            try:
                self.maxGrasp = self.chosenGrasps[0]
                print "max grasp is ", self.maxGrasp, " at ", self.maxGrasp['location']
                print "score of grasp is ", self.maxGrasp['score']

                self.chosenItem = self._most_likely_item(self.maxGrasp)
                print "most likely item is ", self.chosenItem

                #TODO move this outside of if statement
                self.store.put('/robot/target_grasp', self.maxGrasp)
                self.store.put('/robot/grasp_location', self.maxGrasp['location'])

                if(self.store.get('/item/'+self.chosenItem+'/order') is not None):
                    self.store.put('/robot/target_box', self.store.get('/item/'+self.chosenItem+'/order').replace('order', 'box'))
                    self.store.put('/robot/selected_bin', self.store.get('/item/'+self.chosenItem+'/location'))
                #else:
                #    self.store.put('/robot/target_box', 'boxK3')
                #    self.store.put('/robot/selected_bin', self.store.get('/item/'+self.chosenItem+'/location'))

            except (IndexError, RuntimeError) as e:
                self.store.put(['failure', self.getFullName()], "NoGraspsFound")
                logger.exception('no more grasps to try')
                self.setOutcome(False)


        elif self.alg == 'stow':

            self.url = ['photos', 'stow_tote', 'stow']
            self.graspStow = self.store.get(self.url + ['vacuum_grasps'])

            # forget old grasps
            self._expire_grasp_attempts('stow_tote')

            # reject repeat grasps
            self.grasps = [g for g in self.graspStow if not self._check_failed_grasp(failed_grasps, g)]
            if not self.grasps:
                raise RuntimeError('all grasps were rejected')

            self.grasps.sort(key=lambda l: -l['score'])

            try:
                self.maxGrasp = self.grasps[0]
                print "max grasp is ", self.maxGrasp, " at ", self.maxGrasp['location']
                print "score of grasp is ", self.maxGrasp['score']

                self.chosenItem = self._most_likely_itemStow(self.maxGrasp)
                print "most likely item is ", self.chosenItem

                #TODO move this outside of if statement
                self.store.put('/robot/target_grasp', self.maxGrasp)
                self.store.put('/robot/grasp_location', 'stow_tote')

            except (IndexError, RuntimeError) as e:
                self.store.put(['failure', self.getFullName()], "NoGraspsFound")
                logger.exception('no more grasps to try')
                self.store.put('/robot/failed_grasps', [])
                self.setOutcome(False)

        elif self.alg == 'final':
            #self.points =
            pass

        else:
            message = "Algorithm for selection is None, pick, stow, or final"
            logger.error(message)
            raise RuntimeError(message)

        self._mark_grasp_attempt()

        logger.debug("Chosen item is {} worth {} points".format(self.chosenItem, self.store.get('/item/'+self.chosenItem+'/point_value')))
        self.store.put('/robot/selected_item', self.chosenItem)

        self.setOutcome(self.chosenItem is not None)

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None):
            return 0
            #no failure detected, no suggestions!
        elif(self.whyFail == "NoGraspsFound"):
            return 1
            #go to first fallback state
        else:
            return 0
            #again, no suggestions!

    def _screen_chosen_items(self, grasps):
        #self.chosenGrasps = [l for l in self.grasps if self.store.get('/item/'+self._most_likely_item(l)+'/order') is not None]
        good_grasps = []
        for g in self.grasps:
            possible_items = self._get_top_three(g)
            for i in possible_items:
                if(self.store.get('/item/'+i+'/order') is not None):
                    good_grasps.append(g)
                    logger.debug("Adding {} to good_grasps since detected as {}".format(g, i))
                    break
        return good_grasps

    def _expire_grasp_attempts(self, location):
        memory = self.store.get(['planner', 'grasp_memory'], 10)

        failed_grasps = self.store.get(['robot', 'failed_grasps'], [])
        for grasp in failed_grasps:
            if grasp['location'] == location:
                grasp['age'] = grasp.get('age', 0) + 1

        remembered_grasps = []
        for grasp in failed_grasps:
            if grasp['age'] < memory:
                remembered_grasps.append(grasp)
            else:
                logger.info('forgot grasp in {location} at {center} (age {age})'.format(**grasp))

        self.store.put(['robot', 'failed_grasps'], remembered_grasps)

    def _mark_grasp_attempt(self):
        failed_grasps = self.store.get(['robot', 'failed_grasps'], [])

        target_grasp = self.store.get(['robot', 'target_grasp'])
        target_grasp['age'] = 0

        failed_grasps.append(target_grasp)
        self.store.put(['robot', 'failed_grasps'], failed_grasps)

        logger.info('attempted grasp in {location} at {center}'.format(**target_grasp))

    def _check_failed_grasp(self, prior_grasps, new_grasp):
        '''
        Compare grasp positions to tolerance to determine similarity.
        '''

        tolerance = self.store.get('/planner/grasp_failure_radius', 0.05)
        for grasp in prior_grasps:
            distance = ((grasp['center'] - new_grasp['center'])**2).sum()**0.5
            if distance < tolerance:
                logger.info('rejected grasp in {location} at {center}'.format(**new_grasp))
                return True

        return False

    def _most_likely_item(self, g):
        seg = g['segment_id']-1
        IDseg = self.store.get('/photos/'+g['location']+'/tcp/detections')[seg]
        item = max(IDseg, key=lambda l: IDseg[l])
        return item

    def _most_likely_itemStow(self, g):
        seg = g['segment_id']-1
        IDseg = self.store.get('/photos/'+g['location']+'/stow/detections')[seg]
        item = max(IDseg, key=lambda l: IDseg[l])
        return item

    def _get_top_three(self, g):
        seg = g['segment_id']-1
        IDseg = self.store.get('/photos/'+g['location']+'/tcp/detections')[seg]
        sorted_detect = sorted(IDseg.items(), key=lambda x: x[1], reverse=True)
        return [x[0] for x in sorted_detect[0:3]]

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'si')
    SelectItem(myname).run()
