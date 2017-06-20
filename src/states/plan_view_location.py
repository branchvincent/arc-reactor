import logging
from master.fsm import State
from motion.planner import MotionPlanner
from hardware.control.robotcontroller import RobotController
logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanViewLocation(State):
    """
    Input:
        - /robot/target_location: name of location to view
        - /vantage/<target_location>: end-effector's pose for viewing location
    Output:
        - /robot/target_pose: same as /vantage/<target_location> (NOTE: needed?)
        - /robot/waypoints: list of milestones
        - /robot/timestamp: time of route generation
        - /status/route_plan: boolean of motion plan's success
        - /failure/plan_pick_item: failure string
    Failure Cases:
        - infeasible: /vantage/<target_location> is not a feasible pose
    Dependencies
        - None
    """

    def run(self):
        # Get inputs
        self.loc_name = self.store.get('/robot/target_location')
        if self.loc_name is None:
            raise RuntimeError('/robot/target_location is none')

        vantage_T = self.store.get(['vantage', self.loc_name])
        if vantage_T is None:
            raise RuntimeError('/vantage/{} is none'.format(vantage_T))
        self.store.put('/robot/target_pose', vantage_T)

        # Plan route
        p = MotionPlanner(store=self.store)
        p.planToTransform(vantage_T, space='joint', solvers=['local', 'global'])
        p.put()
        success = self.store.get('status/route_plan')
        if not success:
            self.store.put('failure/plan_view_location', 'infeasible')
        self.setOutcome(success)

    def setLoc(self, myname):
        if len(myname) == 4:
            print "Moving to ", 'bin'+myname[-1].upper()
            self.store.put('/robot/target_location', 'bin'+myname[-1].upper())
        elif len(myname) == 5:
            if myname[-2:]=='st':
                print "Moving to stow tote"
                self.store.put('/robot/target_location', 'stow_tote')
            elif myname[-2:]=='at':
                print "Moving to amnesty tote"
                self.store.put('/robot/target_location', 'amnesty_tote')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'pvl')
    PVL = PlanViewLocation(myname)
    PVL.setLoc(myname)
    PVL.run()
