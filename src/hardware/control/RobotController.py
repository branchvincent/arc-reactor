# For executing trajectories

import sys; sys.path.append('../..')

import logging; logger = logging.getLogger(__name__)
from time import sleep
from math import degrees
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum.vpower import PowerUSBStrip, PowerUSBSocket

# List of robot IP addresses
robots = {  'left': '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

class Robot:
    """A robot defined by its trajectory client"""
    def __init__(self, robot='left', port=1000):
        assert(robot in robots)
        self.client = TrajClient(host=robots[robot], port=port)
        self.name = self.client.name
        self.dof = 6
        # self.strip = PowerUSBStrip().strips()[0]
        # self.strip.open()
        # self.socket = PowerUSBSocket(self.strip,1)
        # self.socket.power = 'off'

class Trajectory:
    """A robot trajectory defined by the robot and list of milestones"""
    def __init__(self, robot, milestones, speed=1):
        self.robot = robot
        self.milestones = milestones
        self.curr_index = None
        self.curr_milestone = None
        self.speed = speed
        self.complete = False
        # print milestones

    def processMilestones(self, milestones):
        for m in milestones:
            m[0] /= float(self.speed)
            m[1]['robot'] = [math.degrees(qi) for qi in m[1]['robot']]
            m[1]['robot'][2] = ??
            m[1]['robot'][4] = ??
        return raw_milestones

    def start(self):
        """Sets the current milestone to the first in the list"""
        self.curr_index = 0
        self.curr_milestone = self.milestones[0]
        for m in self.milestones:
            dt = m[0]/float(speed)
            q = m[1]['robot']
            if len(q) != self.robot.dof:
                logger.error('Incorrect robot configuration length.')
                raise Exception('Incorrect robot configuration length.')
            self.robot.client.addMilestone(dt,q)

    def update(self):
        """Checks the current milestone and updates, if necessary"""
        actual_index = len(self.milestones) - self.robot.client.getCurSegments()
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
    def __init__(self, robot='left', store=PensiveClient().default(), speed=1):
        # Robot
        self.robot = Robot(robot)
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
        self.trajectory = Trajectory(self.robot, self.store.get(path))

    def updateTrajectoryExecution(self):
        """Monitors and updates the execution of the robot's trajectory"""
        status = self.trajectory.update(self.robot)
        if status == 'updated':
            # if self.trajectory.vaccumOn():
            #     self.sock.power = 'on'
            # else:
            #     self.socket.power = 'off'
            print "Segment done"

    def testTriangleWave(self,joint=2,deg=10,period=4):
        q1 = self.robot.client.getEndConfig()
        q2, q3 = q[:], q2[:]
        q2[joint] += deg
        q3[joint] -= 2*deg

        # Triangle wave
        milestones = [
               (period*0.5, {
                  'robot': q2,
                  'gripper': [0,0,0],
                  'vaccum': [0]
                })
            ]

        milestones.append((period, {
                   'robot': q3,
                   'gripper': [0,0,0],
                   'vaccum': [0]
                 })
            )

        milestones.append((period*0.5, {
                   'robot': q1,
                   'gripper': [0,0,0],
                   'vaccum': [0]
                 })
            )

        self.store.put('robot/waypoints', milestones)
        self.run()

    def loop(self):
        """The loop executed at the given frequency"""
        while not self.trajectory.complete:
            self.updateCurrentConfig()
            self.updateTrajectoryExecution()
            sleep(0.1)
        print "Trajectory completed"

    def run(self):
        """Runs the current trajectory in the database"""
        self.updatePlannedTrajectory()
        self.trajectory.start()
        self.loop()
        # print "current milestone", self.robot.getCurSegments()
        # print "milestones ",  self.robot.getRemainingSegments()
        # print "max segments ", self.robot.getMaxSegments()

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
