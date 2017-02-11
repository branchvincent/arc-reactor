from master.fsm import State
from hardware.control.RobotController import RobotController
from hardware.control.SimulateRobotController import SimulateRobotController

import logging
logger = logging.getLogger(__name__)

class ExecRoute(State):
    def run(self):
        self.waypoints = self.store.get('/robot/waypoints')
        self.speed = self.store.get('/robot/speed_scale')

        #check if in simulation mode
        if self.store.get('/simulate/robot_motion'):
            try:
                controllerSim = SimulateRobotController()
                controllerSim.run()
            except RuntimeError as e:
                print "Runtime error: ", e
            completed = True  #always pass in sim mode for now
        else:
            try:
                controller = RobotController() # self.waypoints, self.speed
                controller.run()
                completed = controller.trajectory.complete
            except RuntimeError:
                logger.exception('Robot controller had an exception.')
                completed = False

        self.store.put('/status/route_exec', completed)

        #update history
        if completed:
            self.store.put('/robot/waypoints', None)
            self.store.put('/robot/history', self.waypoints)
