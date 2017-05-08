from pensive.client import PensiveClient
from motion.linear_planner import LinearPlanner
from hardware.control.robotcontroller import RobotController
from master.world import xyz,rpy
import math

def getTransform(planner, theta):
    ee_link = 6
    T0 = planner.robot.link(ee_link).getTransform()
    return T0 * rpy(*theta)

if __name__ == "__main__":
    ee_link = 6
    store = PensiveClient().default()
    p = LinearPlanner()
    T0 = p.robot.link(ee_link).getTransform()
    # controller = RobotController()

    # Get all transforms
    T = []
    theta = math.pi/8
    T.append(T0)
    T.append(T0*rpy(theta,0,0))
    T.append(T0*rpy(-theta,0.0))
    T.append(T0*rpy(0,theta,0))
    T.append(T0*rpy(0,-theta,0))
    T.append(T0*rpy(-theta,-theta,0))
    T.append(T0*rpy(theta,theta,0))

    for Ti in T:
        # Plan
        q = p.getDesiredConfig(Ti)
        p.interpolate(q)

        # Execute
        question = lambda: str(raw_input("Execute path? (y/n): ")).lower().strip()[0]
        execute = question()
        while execute not in ['y','n']:
           execute = question()

        if execute == 'y':
            print "Executing..."
            controller = RobotController()
            controller.run()
            # sleep(0.5)
            # print "Reached config. Taking image..."
            # get_point_cloud(bin, order)
        else:
            print "Not executing..."
    # c = SimulatedRobotController(store=store)
    # c.jogTo([0]*7)
