from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class SelectItem(State):
    def run(self):
        self.alg = self.store.get('/robot/task')
        self.itemList = self.store.get('/item/')
        logger.info("Algorithm is {}".format(self.alg))
        if self.alg == None:
            self.chosenItem = max(self.itemList, key=lambda l: self.itemList[l]['point_value'])

        elif self.alg == 'pick':
            #works under condition that loadOrder has been run. Implement check.
            self.chosenItem = max(self.itemList, key=lambda l: self.itemList[l]['point_value'])
            self.store.put('/robot/selected_box', self.store.get('/item/'+self.chosenItem+'/order').replace('order', 'box'))
            self.store.put('/robot/selected_bin', self.store.get('/item/'+self.chosenItem+'/location'))

        elif self.alg == 'stow':
           
            self.url = self.store.get('/robot/target_photo_url')

            # get graspability vector from photo-url vacuum_grasps
            #TODO expand to other grasps
            self.grasps = self.store.get(self.url + ['vacuum_grasps'])
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

        logger.info("Chosen item is {} worth {} points".format(self.chosenItem, self.store.get('/item/'+self.chosenItem+'/point_value')))
        self.store.put('/robot/selected_item', self.chosenItem)

        self.setOutcome(self.chosenItem is not None)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'si')
    SelectItem(myname).run()
