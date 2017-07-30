from .common.plan_stow import PlanStowBase

import logging

logger = logging.getLogger(__name__)

class PlanPlaceBox(PlanStowBase):
    """
    Input:
        - /robot/selected_item: name of item to be placed
        - /robot/placement/pose: end-effector's pose for item placement
        - /robot/placement/location: name of target box (NOTE: needed?)
        - /robot/active_gripper: gripper to be used (vacuum or mechanical) (NOTE: not yet used)
    Output:
        - /failure/plan_place_box: failure string
    Failure Cases:
        - infeasible: /robot/placement/pose is not a feasible placement
    Dependencies:
        - evaluate_placement
    """

    def run(self):
        self._common()

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        check = self.store.get('/status/ppb_done', False)
        counter = self.store.get('/status/ppb_count', 0)
        if(not check):
            self.store.put('/status/ppb_done', True)
            return 0 #try once more
        else:
            if(counter<2):
                self.store.put('/status/ppb_count', 1)
                return 1  #by default to not get stuck in loops
            else:
                self.store.put('/status/ppb_count', 0)
                return 2

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ppb')
    PlanPlaceBox(myname).run()
