from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class SelectItem(State):
    def run(self):
        self.alg = self.store.get('/robot/task')
        self.itemList = self.store.get('/item/')
        logger.info("Algorithm is {}".format(self.alg))

        failed_grasps = self.store.get('/robot/failed_grasps', [])

        if self.alg == None:
            self.chosenItem = max(self.itemList, key=lambda l: self.itemList[l]['point_value'])

        elif self.alg == 'pick':
            self.maxGrasp = None

            # find the best grasp from all the bins
            for name in ['binA', 'binB', 'binC']:
                url = ['photos', name, 'tcp']

                # get graspability vector from photo-url vacuum_grasps
                #TODO expand to other grasps
                self.grasps = self.store.get(url + ['vacuum_grasps'])
                # exlcude prior grasps from consideration
                #self.grasps = [g for g in self.grasps if not self.check_similar_grasp(failed_grasps, g)]

                maxGrasp = max(self.grasps, key=lambda l: l['score'])

                if self.maxGrasp:
                    if maxGrasp['score'] < self.maxGrasp['score']:
                        continue

                self.maxGrasp = maxGrasp
                self.url = url
                print "max grasp is ", self.maxGrasp, " at ", self.url
                self.maxGraspSeg = self.maxGrasp['segment_id'] - 1

            # match greatest graspability segment to the highest likely item ID
            self.idSeg = self.store.get(self.url + ['detections'])[self.maxGraspSeg]
            self.chosenItem = max(self.idSeg, key=lambda l: self.idSeg[l])

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

            self.url = self.store.get('/robot/target_photo_url')

            # get graspability vector from photo-url vacuum_grasps
            #TODO expand to other grasps
            self.grasps = self.store.get(self.url + ['vacuum_grasps'])
            # exlcude prior grasps from consideration
            self.grasps = [g for g in self.grasps if not self.check_similar_grasp(failed_grasps, g)]

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

        failed_grasps.append(self.maxGrasp)
        self.store.put('/robot/failed_grasps', failed_grasps)

        logger.info("Chosen item is {} worth {} points".format(self.chosenItem, self.store.get('/item/'+self.chosenItem+'/point_value')))
        self.store.put('/robot/selected_item', self.chosenItem)

        self.setOutcome(self.chosenItem is not None)

    def check_similar_grasp(self, prior_grasps, new_grasp, tolerance=None):
        '''
        Compare grasp positions to tolerance to determine similarity.
        '''
        # HACK: ignored repeated grasps
        return False

        tolerance = tolerance or self.store.get('/planner/similar_grasp_tolerance', 0.01)
        for grasp in prior_grasps:
            distance = sum([(x - y)**2 for (x, y) in zip(grasp['position'], new_grasp['position'])])
            if distance < tolerance**2:
                return True

        return False

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'si')
    SelectItem(myname).run()
