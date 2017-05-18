from master.fsm import State
from hardware.control.robotcontroller import RobotController
from hardware.control.simulatedrobotcontroller import SimulatedRobotController

import logging
logger = logging.getLogger(__name__)

class ExecRoute(State):
    def run(self):
        # self.waypoints = self.store.get('/robot/waypoints')
        # self.speed = self.store.get('/robot/speed_scale', 1)

        #check if in simulation mode
        if self.store.get('/simulate/robot_motion'):
            try:
                controllerSim = SimulatedRobotController(store=self.store)
                controllerSim.run()
            except RuntimeError as e:
                print "Runtime error: ", e
            completed = True  #always pass in sim mode for now
        else:
            try:
                controller = RobotController(store=self.store)
                controller.run()
                completed = controller.trajectory.complete
                self.setOutcome(True)
            except RuntimeError:
                self.setOutcome(False)
                logger.exception('Robot controller had an exception.')
                completed = False

        #update history
        # if completed:
        #     self.store.put('/robot/waypoints', None)
        #     self.store.put('/robot/history', self.waypoints)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'er')
    ExecRoute(myname).run()
