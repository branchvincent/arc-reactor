from pensive.client import PensiveClient
from motion.linear_planner import LinearPlanner
from hardware.control.robotcontroller import RobotController
from hardware.control.simulatedrobotcontroller import SimulatedRobotController
from master.world import xyz,rpy,klampt2numpy,numpy2klampt
import logging; logger = logging.getLogger(__name__)

from klampt.math import so3

import math
import time

class CalibrateCam:
    def __init__(self, x_target):
        self.store = PensiveClient().default()
        self.planner = LinearPlanner()
        self.x_target = x_target
        self.robot = self.planner.robot

    def run(self):
        # Get all transforms
        # Tcurr = self.robot.link(6).getTransform()

        # Move to start
        Rz = math.atan2(self.x_target[1], self.x_target[0]) - math.pi/2
        T0 = numpy2klampt(xyz(*self.x_target) * rpy(math.pi,0,0) * rpy(0,0,-Rz))
        # print 'From \n{} \nto \n{}'.format(Tcurr[1], T0[1])
        self.store.put('robot/inspect_pose', T0)
        self.planner.interpolate(T=T0)
        self.executePlan()

        # Execute transforms
        Ts = self.getTransforms(self.robot, T0)
        for Ti in Ts:
            self.store.put('robot/inspect_pose', Ti)
            self.planner.interpolate(T=Ti)
            # print 'Executing: t = {}'.format([round(tii,2) for tii in Ti[1]])
            self.executePlan()

    def getTransforms(self, robot, T0):
        # Calculate desired transforms
        Ts = []
        Ts += self.getCircle(T0, z=0.85)
        Ts += [T0]
        Ts += self.getCircle(T0, z=0.65)
        Ts += [T0]
        return Ts

    def getCircle(self, T0, r=0.2, z=0.5, N=2):
        R0 = T0[0]
        x0,y0,_ = T0[1]
        s = 2*math.pi/N
        ee = self.robot.link(6)

        Ts = []
        for ni in xrange(0,N):
            Ri = numpy2klampt(klampt2numpy(R0) * rpy(0,0,-s*ni) * rpy(0,math.pi/8,0))[0]
            ti = ee.getWorldPosition([r*math.cos(s*ni), -r*math.sin(s*ni), 0])[:-1] + [z]
            Ts.append((Ri, ti))
        return Ts

    def executePlan(self):
        if self.query():
            print "Executing..."
            SimulatedRobotController().run()
            # RobotController().run()
        else:
            print "Not executing..."

    def query(self):
        question = lambda: str(raw_input("Execute? (y/n): ")).lower().strip()[0]
        execute = question()
        while execute not in ['y','n']:
            execute = question()
        return execute == 'y'

if __name__ == "__main__":
    # x_target = (0.07, 0.77, 0.85)
    x_target = (0, 0.47, 0.85)
    # x_target = (-0.37, 0.37, 0.85)
    cc = CalibrateCam(x_target)
    cc.run()
