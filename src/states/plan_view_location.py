import logging
from master.fsm import State
from motion.linear_planner import LinearPlanner
from hardware.control.robotcontroller import RobotController
logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PlanViewLocation(State):
    def run(self):
        # Get location
        self.loc_name = self.store.get('/robot/target_location')
        vantage_T = self.store.get(['vantage', self.loc_name])
        self.store.put('/robot/target_xform', vantage_T)
        # T = self.store.get('/robot/target_xform') # or /robot/vantage_url?

        if vantage_T is None:
            self.setOutcome(False)
            raise RuntimeError("no target is set to view")
        else:
            # Plan route
            lp = LinearPlanner()
            lp.interpolate(T=vantage_T)
            self.setOutcome(True)

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
