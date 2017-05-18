import logging
from master.fsm import State
from motion.linear_planner import LinearPlanner
from hardware.control.robotcontroller import RobotController
logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class ViewLocation(State):
    def run(self):
        # Get location
        item = self.store.get('robot/selected_item')
        location = self.store.get(['item',item,'location'])

        # Plan route TODO: get desired pose
        lp = LinearPlanner()
        # T = self.store.get('')
        # lp.interpolate(T=T)
        
       self.setOutcome(True)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'vl')
    ViewLocation(myname).run()
