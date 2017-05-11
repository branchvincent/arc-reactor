from pensive.client import PensiveClient
from motion.linear_planner import LinearPlanner
from hardware.control.robotcontroller import RobotController
from hardware.control.simulatedrobotcontroller import SimulatedRobotController
from master.world import xyz,rpy,klampt2numpy,numpy2klampt
import logging; logger = logging.getLogger(__name__)

from klampt.math import so3

import math,time

def getTransforms(robot, T0):
    # Get current transform
    ee_link = 6
    Tcurr = robot.link(ee_link).getTransform()
    print "Tcurr",Tcurr
    # T0 = Tcurr
    # T0 = (Tcurr[0], origin)
    # T0 = klampt2numpy(T0)

    # Calculate desired transforms
    Ts = [T0]
    Ts += getCircle(T0, z=0.85)
    Ts += [T0]
    Ts += getCircle(T0, z=0.65)
    Ts += [T0]
    # T = []
    # theta = math.pi/8
    # T.append(T0)
    # T.append(T0*rpy(theta,0,0))
    # T.append(T0*rpy(-theta,0.0))
    # T.append(T0*rpy(0,theta,0))
    # T.append(T0*rpy(0,-theta,0))
    # T.append(T0*rpy(-theta,-theta,0))
    # T.append(T0*rpy(theta,theta,0))
    return Ts

def getCircle(T0, radius=0.2, z=0.5, N=18):
    def circum(p0, z=z, r=radius, n=N):
        x0,y0,z0 = p0
        s = 2*math.pi/n
        return [(x0 + r*math.cos(s*ni), y0 + r*math.sin(s*ni), z) for ni in xrange(0,n)]
    def angle(R0, r=radius, n=N):
        return [R0 for ni in xrange(0,n)]
    R0,t0 = T0
    Rs,ts = angle(R0), circum(t0)
    return [(Ri,ti) for Ri,ti in zip(Rs,ts)]

def executePlan():
    # Query
    question = lambda: str(raw_input("Execute path? (y/n): ")).lower().strip()[0]
    execute = question()
    while execute not in ['y','n']:
       execute = question()

    # Execute
    if execute == 'y':
        print "Executing..."
        SimulatedRobotController().run()
        # controller = RobotController()
        # controller.run()
    else:
        print "Not executing..."

if __name__ == "__main__":
    # store = PensiveClient().default()
    # Get all transforms
    # o = (0.5,0.2,0.85)
    # R0 = so3.identity()
    # t0 = (0.07,0.77,0.85)
    T0 = rpy(0,math.pi,0) * xyz(0.07,0.77,0.85)
    T0 = numpy2klampt(T0)
    # o = (-0.7,0,0.85)
    # o = [-0.7,0,-0.5]
    p = LinearPlanner()
    Tcurr = p.robot.link(6).getTransform()
    print "From", Tcurr[0]
    print "To",T0
    # o = Tcurr[1]; o[0] += 0.2
    # T0 = [Tcurr[0], t0]
    Ts = getTransforms(p.robot, T0)
    for Ti in Ts:
        print 't', [round(tii,2) for tii in Ti[1]]
    for Ti in Ts:
        print 'Interpolating'
        p.interpolate(T=Ti)
        # try:
        #     p.interpolate(T=Ti)
        # except Exception, e:
        #     logger.warning(e)
        # time.sleep(1)
        print 'Executing'
        print 't', [round(tii,2) for tii in Ti[1]]
        executePlan()
        # time.sleep(1)
