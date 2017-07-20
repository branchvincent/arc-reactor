from master.fsm import State

import logging
from time import time

logger = logging.getLogger(__name__)

class PlanReplaceItem(State):
    """
    Input:
        - /robot/waypoints: list of milestones to be reversed
    Output:
        - /robot/waypoints: list of milestones
        - /robot/timestamp: time of route generation
        - /failure/<state_name>: failure string
    Failure Cases:
        - none
    Dependencies:
        - none
    """

    def run(self):
        # Reverse the last executed plan
        motion_plan = self.store.get('robot/waypoints', [])
        reversed_motion_plan = list(reversed(motion_plan))

        # Update store
        self.store.put('robot/waypoints', reversed_motion_plan)
        self.store.put('robot/timestamp', time())


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ppi')
    PlanReplaceItem(myname).run()
