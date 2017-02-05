# For executing trajectories

import sys; sys.path.append('../..')

import logging; logger = logging.getLogger(__name__)
from time import sleep
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum.vpower import PowerUSBStrip, PowerUSBSocket

robots = {  'left': '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

class Robot:
    """A robot defined by its trajectory client"""
    def __init__(self, robot='left', port=1000):
        print "host ", robots[robot]
        print "port ", port
        self.client = TrajClient(host=robots[robot], port=port)
        self.name = self.client.name
        self.dof = 6

class Trajectory:
    """A robot trajectory defined by a list of milestones"""
    def __init__(self, milestones):
        self.milestones = milestones
        self.curr_index = None
        self.curr_milestone = None
        self.complete = False
        print milestones

    def start(self, robot):
        """Sets the current milestone to the first in the list"""
        self.curr_index = 0
        self.curr_milestone = self.milestones[0]
        for m in self.milestones:
            dt = m[0]
            q = m[1]['robot']
            if len(q) != robot.dof:
                logger.error('Incorrect robot configuration length.')
                raise Exception('Incorrect robot configuration length.')
            robot.client.addMilestone(dt,q)

    def update(self, robot):
        actual_index = len(self.milestones) - robot.client.getCurSegments()
        if (actual_index != self.curr_index):
            self.advanceMilestone()
            return 'updated'

    def advanceMilestone(self):
        """Advances to the next milestone, if applicable"""
        self.curr_index += 1
        if (self.curr_index < len(self.milestones)):
            self.curr_milestone = self.milestones[self.curr_index]
        else:
            self.complete = True
            self.curr_index = None
            self.curr_milestone = None

    def vaccumOn(self):
        return self.curr_milestone[1]['vaccumm']


class RobotController:
    """Trajectory execution for TX90."""
    def __init__(self, robot='left', port=1000, store=PensiveClient().default()):
        # Robot
        self.robot = Robot(robot,port)
        self.trajectory = None
        self.freq = 100
        # USB hub
        # self.strip = PowerUSBStrip().strips()[0]
        # self.strip.open()
        # self.socket = PowerUSBSocket(self.strip,1)
        # self.socket.power = 'off'
        # Database
        self.store = store

    def updateCurrentConfig(self, path='robot/current_config'):
        """Updates the database with the robot's current configuration."""
        self.store.put(path, self.robot.client.getConfig())

    def updatePlannedTrajectory(self, path='robot/waypoints'):
        """Updates the robot's planned trajectory from the database"""
        self.trajectory = Trajectory(self.store.get(path))

    def startTrajectoryExecution(self):
        """Starts to execute the trajectory by adding the milestones to the queue."""
        self.trajectory.start(self.robot)

    def updateTrajectoryExecution(self):
        """Monitors and updates the execution of the robot's trajectory"""
        status = self.trajectory.update(self.robot)
        if status == 'updated':
            # query vaccum
            print "Segment done"
            pass

    def testTriangleWave(self,joint=2,deg=10,period=4):
        q = self.robot.client.getEndConfig()
        q2 = q[:]
        q2[joint] += deg

        #triangle wave
        milestones = [
               (period*0.5, {
                  'robot': q2,
                  'gripper': [0,0,0],
                  'vaccum': [0]
                })
            ]

        q3 = q2[:]
        q3[joint] -= 2*deg
        print q2[joint]

        milestones.append((period, {
                   'robot': q3,
                   'gripper': [0,0,0],
                   'vaccum': [0]
                 })
            )

        milestones.append((period*0.5, {
                   'robot': q,
                   'gripper': [0,0,0],
                   'vaccum': [0]
                 })
            )

        self.store.put('robot/waypoints', milestones)

        self.run()
        return

    def loop(self):
        """"""
        while not self.trajectory.complete:
            self.updateCurrentConfig()
            self.updateTrajectoryExecution()
            sleep(0.1)

        print "Trajectory completed"

    def run(self):
        """"""
        self.updatePlannedTrajectory()
        self.startTrajectoryExecution()
        self.loop()

        # curr_milestone = self.robot.getCurSegments()
        # milestones = self.robot.getRemainingSegments()
        # print "current milestone", curr_milestone
        # print "milestones ", milestones
        # print "max segments ", self.robot.getMaxSegments()

# class Trajectory:
#     """A robot trajectory defined by a list of milestones"""
#     def __init__(self, milestones, robot):
#         self.milestones = milestones
#         self.curr_index = None
#         self.curr_milestone = None
#         self.robot = robot
#     #
#     # def start(self):
#     #     """Starts to execute the trajectory by adding the milestones to the queue."""
#     #     self.curr_index = 0
#     #     self.curr_milestone = self.milestones[0]
#     #     for m in self.milestones:
#     #         dt = m[0]
#     #         q = m[1]['robot']
#     #         if len(q) != robot.dof:
#     #             logger.error('Incorrect robot configuration length.')
#     #             raise Exception('Incorrect robot configuration length.')
#     #         self.robot.client.addMilestone(dt,q)
#     #
#     # def update(self):
#     #     """Monitors and updates the execution of the trajectory"""
#     #     actual_milestone = robot.client.getCurSegments()
#     #     belief_milestone = [self.curr_milestone[0], self.curr_milestone[1]['robot']]
#     #     if (actual_milestone != belief_milestone):
#     #         success = self.advanceMilestone()
#     #         return success
#     #     else:
#     #         return true
#     #
#     # def advanceMilestone(self):
#     #     """Advances to the next milestone, if applicable"""
#     #     self.curr_index += 1
#     #     if (self.curr_index < len(self.milestones)):
#     #         self.curr_milestone = self.milestones[self.curr_index]
#     #         if self.milestones[1]['vaccumm']
#     #         return true
#     #     else:
#     #         self.curr_index = None
#     #         self.curr_milestone = None
#     #         return false
#     #
#     # def end(self):
#     #     """"""
#     #     pass
#     #
#     # def run(self):
#     #     self.start()
#     #     self.update()

if __name__ == "__main__":
    store = PensiveClient(host='http://10.10.1.102:8888/').default()
    c = RobotController(store=store)
    sample_milestones = [
           (2, {
              'robot': [0, 0, 0, 0, 0, 0],
              'gripper': [0,0,0],
              'vaccum': [0]
            })
        ]
    store.put('robot/waypoints', sample_milestones)
    # c.run()
    c.testTriangleWave()
