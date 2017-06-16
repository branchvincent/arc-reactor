from master.fsm import State
from hardware.control.robotcontroller import RobotController
from hardware.control.simulatedrobotcontroller import SimulatedRobotController

import logging
logger = logging.getLogger(__name__)

class ExecRoute(State):
    """
    Input:
        - /robot/waypoints (optional): list of milestones
        - /robot/speed_scale (optional): speed scale for milestones
        - /robot/camera_xform: local pose of end effector camera
    Output:
        - /robot/tcp_pose: updated pose of end effector
        - /camera/tcp/pose: updated pose of end effector camera
        - /robot/current_config: udpated robot configuration
        - /failure/exec_route: failure string
    Failure Cases:
        - connection_error: could not connect to robot (TODO: classify)
    Dependencies:
        - a planning state
    """

    def run(self):
        #check if in simulation mode
        if self.store.get('/simulate/robot_motion', True):
            try:
                controllerSim = SimulatedRobotController(store=self.store)
                controllerSim.run()
            except RuntimeError as e:
                print "Runtime error: ", e
            self.setOutcome(True)
        else:
            try:
                controller = RobotController(store=self.store)
                controller.run()
                completed = controller.trajectory.complete
                self.setOutcome(True)
            except RuntimeError:
                # TODO: this may not be the error (we should add a timeout feature to controller)
                self.store.put('failure/exec_route', 'connection_error')
                logger.exception('Robot controller had an exception.')
                self.setOutcome(False)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('name', nargs='?')
    args = parser.parse_args()
    myname = (args.name or 'er')
    ExecRoute(myname).run()
