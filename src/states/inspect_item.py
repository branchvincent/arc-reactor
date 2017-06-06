from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class InspectItem(State):
    def run(self):

        #do stuff

        #and then set up where to go next
        task = self.store.get('/robot/task', 'stow')
        if task == 'stow':
            self.store.put('/robot/target_locations', ['binA', 'binB', 'binC'])
        else: self.store.put('/robot/target_locations', 'stow_tote')
        self.setOutcome(True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ii')
    InspectItem(myname).run()

