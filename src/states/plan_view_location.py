import logging
from master.fsm import State
from motion.planner import MotionPlanner
logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanViewLocation(State):
    """
    Input:
        - /robot/target_view_location: name of location to view
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
        self.loc_name = self.store.get('/robot/target_view_location')
        if self.loc_name is None:
            raise RuntimeError('/robot/target_view_location is none')

        vantage_T = self.store.get(['vantage', self.loc_name])
        if vantage_T is None:
            raise RuntimeError('/vantage/{} is none'.format(self.loc_name))
        self.store.put('/robot/target_pose', vantage_T)

        # Plan route
        self.store.put('planner/current_state', 'idle')
        planner = MotionPlanner(store=self.store)
        planner.toTransform(vantage_T)
        motion_plan = self.store.get('robot/waypoints')
        if motion_plan is not None:
            self.store.put('/status/pvl_done', False)
            self.setOutcome(True)
        else:
            self.store.put(['failure', self.getFullName()], "infeasible")
            self.setOutcome(False)

    def setLoc(self, myname):
        if len(myname) == 4:
            print "Moving to ", 'bin'+myname[-1].upper()
            self.store.put('/robot/target_view_location', 'bin'+myname[-1].upper())
        elif len(myname) == 5:
            if myname[-2:]=='st':
                print "Moving to stow tote"
                self.store.put('/robot/target_view_location', 'stow_tote')
            elif myname[-2:]=='at':
                print "Moving to amnesty tote"
                self.store.put('/robot/target_view_location', 'amnesty_tote')

    def suggestNext(self):
        self.whyFail = self.store.get(['failure', self.getFullName()])
        if(self.whyFail is None):
            #unknown error, check to see if this has happened already
            check = self.store.get('/status/pvl_done', False)
            if(check):
                return 1
            else:
                self.store.put('/status/pvl_done', True)
                return 0
        elif(self.whyFail == "infeasible"):
            return 1
        else:
            return 1  #by default to not get stuck in loops

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'pvl')
    PVL = PlanViewLocation(myname)
    PVL.setLoc(myname)
    PVL.run()
