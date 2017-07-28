import logging

from math import pi

from master.fsm import State
from master.world import rpy

from motion.planner import MotionPlanner

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
        for angle in [0, pi]:
            self.store.put('/robot/inspect_pose', self.store.get('/robot/inspect_pose').dot(rpy(0, 0, angle)))

            inspect_pose[2, 3] += self.store.get('/planner/swivel_local')[2]

            planner = MotionPlanner(store=self.store)
            planner.toTransform(inspect_pose, swivel=0)
            motion_plan = self.store.get('robot/waypoints')
            if motion_plan is not None:
                self.setOutcome(True)
                break
        else:
            self.store.put(['failure', self.getFullName()], 'infeasible')
            self.setOutcome(False)

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        check = self.store.get('/status/pis_done', False)
        if(not check):
            self.store.put('/status/pis_done', True)
            return 0 #try once more
        else:
            return 1  #by default to not get stuck in loops

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'pis')
    pis = PlanInspectionStation(myname)
    pis.run()
