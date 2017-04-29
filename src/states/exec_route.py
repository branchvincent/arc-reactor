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
                controllerSim = SimulatedRobotController(store=self.store) #, milestones=self.waypoints, speed=self.speed)
                controllerSim.run()
            except RuntimeError as e:
                print "Runtime error: ", e
            completed = True  #always pass in sim mode for now
        else:
            try:
                controller = RobotController(store=self.store) #, milestones=self.waypoints, speed=self.speed) # self.waypoints, self.speed
                controller.run()
                completed = controller.trajectory.complete
            except RuntimeError:
                logger.exception('Robot controller had an exception.')
                completed = False

        self.store.put('/status/route_exec', completed)

        #update history
        # if completed:
        #     self.store.put('/robot/waypoints', None)
        #     self.store.put('/robot/history', self.waypoints)

if __name__ == '__main__':
    ExecRoute('er').run()
