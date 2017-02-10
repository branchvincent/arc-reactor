from master.fsm import State
from hardware.control import RobotController

class ExecRoute(State):
    def run(self):
        self.waypoints = self.store.get('/robot/waypoints')
        self.speed = self.store.get('/robot/speed_scale')

        #move along waypoints list
        controller = RobotController() # self.waypoints, self.speed
        controller.run()
        completed = controller.trajectory.complete

        #figure out where we are now
        self.store.put('/status/route_exec', completed)

        #update history
        if completed:
            self.store.put('/robot/waypoints', None)
            self.store.put('/robot/history', self.waypoints)
