import logging
import argparse
from master.fsm import State
from checkpoint import grasp

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class CheckGrasp(State):
    def run(self):
        self.setOutcome(grasp.run(locations=[]))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'cg')
    CheckGrasp(myname).run()
