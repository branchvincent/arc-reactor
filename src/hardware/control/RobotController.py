# For executing trajectories

import sys; sys.path.append('../..')

import logging; logger = logging.getLogger(__name__)
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum.vpower import PowerUSBStrip, PowerUSBSocket

leftarm = '10.10.1.202'
rightarm = '10.10.1.203'
localarm = 'localhost'

class RobotController:
    """Trajectory execution for TX90."""
    def __init__(self, host=leftarm, port=1000, store=PensiveClient().default()):
        # Robot
        self.robotHost = host
        self.robot = TrajClient(host=host,port=port)
        self.dof = 6
        # USB
        self.strip = PowerUSBStrip().strips()[0]
        self.socket = PowerUSBSocket(self.strip,1)
        # Database
        self.store = store

    def updateCurrentConfig(self, path='robot/current_config'):
        """Update the database with the robot's current configuration."""
        self.store.put(path, self.robot.getConfig())

    def updateGoalConfig(self, path='robot/goal_config'):
        """Update the database with the robot's goal configuration."""
        self.store.put(path, self.robot.getEndConfig())

    def getAndAddMilestones(self, path='robot/waypoints'):
        """Get milestones from the database and add to the motion queue."""
        milestones = self.store.get(path)
        for m in milestones:
            dt = m[0]
            q = m[1]['robot']
            if len(q) != self.dof:
                logger.error('Incorrect robot configuration length.')
                raise Exception('Incorrect robot configuration length.')
            print "moving robot"

            self.robot.addMilestone(dt,q)

    def addMilestone(self,dt,q):
        """Add a milestone to the motion queue."""
        self.robot.addMilestone(dt,q)

    def addMilestones(self,dts,qs):
        """Add milestones to the motion queue."""
        self.robot.appendMilestones(dts,qs)

    def testTriangleWave(self,joint=2,deg=10,period=4):
        q = self.robot.getEndConfig()
        q2 = q[:]
        q2[joint] += deg
        #triangle wave
        self.robot.addMilestone(period*0.5,q2)
        q2[joint] -= 2*deg
        self.robot.addMilestone(period,q2)
        self.robot.addMilestone(period*0.5,q)
        return

    def run(self):
        """"""
        self.getAndAddMilestones()

        # curr_milestone = self.robot.getCurSegments()
        # milestones = self.robot.getRemainingSegments()
        # print "current milestone", curr_milestone
        # print "milestones ", milestones
        # print "max segments ", self.robot.getMaxSegments()

if __name__ == "__main__":
    c = RobotController()
    milestones = [
           (2, {
              'robot': [0, 0, 0, 0, 0, 0],
              'gripper': [0,0,0],
              'vaccum': [0]
            })
        ]
    c.store.put('robot/waypoints', milestones)
    # c.run()
    c.testTriangleWave()
