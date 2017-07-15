from master.fsm import State
from motion.planner import MotionPlanner

import logging

logger = logging.getLogger(__name__)

class PlanInspectionStation(State):
    """
    Input:
        - /robot/inspect_pose: pose of inspection station
    Output:
        - /robot/waypoints: list of milestones
        - /robot/timestamp: time of route generation
        - /status/route_plan: boolean of motion plan's success
        - /failure/<state_name>: failure string
    Failure Cases:
        - infeasible: /robot/inspect_pose is not a feasible pose
    Dependencies
        - None
    """

    def run(self):
        # Get inspection pose
        inspect_pose = self.store.get(['robot', 'inspect_pose'])
        if inspect_pose is None:
            raise RuntimeError('/robot/inspect_pose/ is none')

        # Plan route
        self.store.put('planner/current_state', 'carrying')
        planner = MotionPlanner(store=self.store)
        planner.toTransform(inspect_pose)
        motion_plan = self.store.get('robot/waypoints')
        if motion_plan is not None:
            self.setOutcome(True)
        else:
            self.store.put(['failure', self.getFullName()], 'infeasible')
            self.setOutcome(False)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'pvl')
    pis = PlanInspectionStation(myname)
    pis.setLoc(myname)
    pis.run()
