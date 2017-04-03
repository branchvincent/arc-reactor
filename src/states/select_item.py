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
            # XXX: copied from picking for testing purposes
            self.chosenItem = self.itemList.keys()[0]
            #self.points =
            pass

        elif self.alg == 'final':
            #self.points =
            pass

        else:
            message = "Algorithm for selection is None, pick, stow, or final"
            logger.error(message)
            raise RuntimeError(message)

        logger.info("Chosen item is {} worth {} points".format(self.chosenItem, self.store.get('/item/'+self.chosenItem+'/point_value')))
        self.store.put('/robot/selected_item', self.chosenItem)

        self.store.put('/status/selected_item', self.chosenItem is not None)
