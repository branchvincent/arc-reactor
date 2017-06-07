from master.fsm import State
import logging; logger = logging.getLogger(__name__)

class InspectItem(State):
    def run(self):

        #do stuff

        #and then set up where to go next
        task = self.store.get('/robot/task')
        if task is None:
            raise RuntimeError("No task defined")
        elif task == 'stow':
            self.store.put('/robot/target_locations', ['binA', 'binB', 'binC'])
        elif task=='pick':
            self.store.put('/robot/target_locations', self.store.get('/robot/selected_box'))

        self.setOutcome(True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ii')
    InspectItem(myname).run()

