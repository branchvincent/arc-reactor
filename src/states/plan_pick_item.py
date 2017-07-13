from .common.plan_pick import PlanPickBase

import logging

logger = logging.getLogger(__name__)

class PlanPickItem(PlanPickBase):
    """
    Input:
        - /robot/selected_item: name of item to be picked
        - /robot/target_grasp: dictionary containing item's grasp information
        - /robot/active_gripper: name of gripper to be used (vacuum or mechanical) (NOTE: not yet used)
    Output:
        - /robot/waypoints: list of milestones
        - /robot/timestamp: time of route generation
        - /failure/plan_pick_item: failure string
    Failure Cases:
        - infeasible: /robot/target_grasp is not a feasible grasp
    Dependencies:
        - selected_item
    """

    def run(self):
        self._common()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'ppi')
    PlanPickItem(myname).run()
