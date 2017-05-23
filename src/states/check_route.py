import logging
import argparse
from master.fsm import State
from checkpoint import motion_plan

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class CheckRoute(State):
    def run(self):
        self.setOutcome(motion_plan.run())

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'cr')
    CheckRoute(myname).run()
