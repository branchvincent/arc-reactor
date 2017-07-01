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
            
            self.graspBins = self.graspsA + self.graspsB + self.graspsC

            self.grasps = [g for g in self.graspBins if not self._check_failed_grasp(failed_grasps, g)]

            self.grasps.sort(key=lambda l: -l['score'])

            self.chosenGrasps = [l for l in self.grasps if self.store.get('/item/'+self._most_likely_item(l)+'/order') is not None]

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
            #HACK to put something in a bpx for now
            else:
                self.store.put('/robot/target_box', 'boxK3')
                self.store.put('/robot/selected_bin', self.store.get('/item/'+self.chosenItem+'/location'))

        elif self.alg == 'stow':

            self.url = self.store.get('/robot/target_photos_url')

            # get graspability vector from photo-url vacuum_grasps
            #TODO expand to other grasps
            self.grasps = self.store.get(self.url + ['vacuum_grasps'])
            # exlcude prior grasps from consideration
            self.grasps = [g for g in self.grasps if not self._check_failed_grasp(failed_grasps, g)]

            self.maxGrasp = max(self.grasps, key=lambda l: l['score'])
            self.maxGraspSeg = self.maxGrasp['segment_id'] - 1

            # match greatest graspability segment to the highest likely item ID
            self.idSeg = self.store.get(self.url + ['detections'])[self.maxGraspSeg]
            self.chosenItem = max(self.idSeg, key=lambda l: self.idSeg[l])

            #TODO move this outside of if statement
            self.store.put('/robot/target_grasp', self.maxGrasp)
            self.store.put('/robot/grasp_location', 'stow_tote')

        elif self.alg == 'final':
            #self.points =
            pass

        else:
            message = "Algorithm for selection is None, pick, stow, or final"
            logger.error(message)
            raise RuntimeError(message)

        self._mark_grasp_attempt()

        logger.info("Chosen item is {} worth {} points".format(self.chosenItem, self.store.get('/item/'+self.chosenItem+'/point_value')))
        self.store.put('/robot/selected_item', self.chosenItem)

        self.setOutcome(self.chosenItem is not None)

    def _mark_grasp_attempt(self):
        failed_grasps = self.store.get(['robot', 'failed_grasps'], [])
        target_grasp = self.store.get(['robot', 'target_grasp'])

        failed_grasps.append(target_grasp)
        self.store.put(['robot', 'failed_grasps'], failed_grasps)

        logger.info('grasp attempted at {}'.format(target_grasp['center']))

    def _check_failed_grasp(self, prior_grasps, new_grasp):
        '''
        Compare grasp positions to tolerance to determine similarity.
        '''
        tolerance = self.store.get('/planner/grasp_failure_radius', 0.05)
        for grasp in prior_grasps:
            distance = ((grasp['center'] - new_grasp['center'])**2).sum()**0.5
            if distance < tolerance:
                logger.info('grasp rejected at {}'.format(grasp['center']))
                return True
        return False

    def _most_likely_item(self, g):
        seg = g['segment_id']-1
        IDseg = self.store.get('/photos/'+g['location']+'/tcp/detections')[seg]
        item = max(IDseg, key=lambda l: IDseg[l])
        return item

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'si')
    SelectItem(myname).run()
