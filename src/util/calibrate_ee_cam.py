from pensive.client import PensiveClient
from motion.linear_planner import LinearPlanner
from hardware.control.robotcontroller import RobotController
from master.world import xyz,rpy,klampt2numpy

import math

def getTransforms(robot, origin):
    # Get current transform
    ee_link = 6
    T0 = robot.link(ee_link).getTransform()
    T0[1] = origin
    T0 = klampt2numpy(T0)

    # Calculate desired transforms
    Ts = [T0]
    Ts += getCircle(T0, z=0.6)
    Ts += getCircle(T0, z=0.4)
    # T = []
    # theta = math.pi/8
    # T.append(T0)
    # T.append(T0*rpy(theta,0,0))
    # T.append(T0*rpy(-theta,0.0))
    # T.append(T0*rpy(0,theta,0))
    # T.append(T0*rpy(0,-theta,0))
    # T.append(T0*rpy(-theta,-theta,0))
    # T.append(T0*rpy(theta,theta,0))
    return T

def getCircle(T0, radius=0.2, z=0.5, N=8):
    def circum(x0, r=radius, n=N):
        x,y,z = x0
        c = 2*math.pi/n
        return [(x + r*math.cos(c*ni), y + r*math.sin(c*ni), z) for ni in xrange(0,n)]
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
        RobotController().run()
        # controller = RobotController()
        # controller.run()
    else:
        print "Not executing..."

if __name__ == "__main__":
    # store = PensiveClient().default()
    # Get all transforms
    o = (1,1,0)
    p = LinearPlanner()
    Ts = getTransforms(p.robot, o)
    for Ti in Ts:
        p.interpolate(T=Ti)
        executePlan()
