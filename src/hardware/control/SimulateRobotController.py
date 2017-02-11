# For executing trajectories

import sys; sys.path.append('../..')

import logging; logger = logging.getLogger(__name__)
from time import sleep, time
from copy import deepcopy
from math import pi, degrees, radians
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum.vpower import PowerUSBStrip, PowerUSBSocket

# List of robot IP addresses
robots = {  'left': '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

def convertConfigToRobot(q_old, speed):
    assert(len(q_old[1]['robot']) == 7)
    assert(0 < speed <= 1)
    q = list(deepcopy(q_old))
    q[0] /= float(speed)
    q[1]['robot'] = [degrees(qi) for qi in q[1]['robot']]
    q[1]['robot'][3] *= -1
    q[1]['robot'][5] *= -1
    del q[1]['robot'][0]
    return tuple(q)

def convertConfigToDatabase(q_old):
    assert(len(q_old[1]['robot']) == 6)
    q = list(deepcopy(q_old))
    q[1]['robot'].insert(0,0)
    q[1]['robot'] = [radians(qi) for qi in q[1]['robot']]
    q[1]['robot'][3] *= -1
    q[1]['robot'][5] *= -1
    return tuple(q)

# class SimulateRobot:
#     """A robot defined by its trajectory client"""
#     def __init__(self, robot='left', port=1000):
#         assert(robot in robots)
#         # self.client = TrajClient(host=robots[robot], port=port)
#         # self.name = self.client.name
#         self.dof = 6
#         # USB
#         self.strip = PowerUSBStrip().strips()[0]
#         self.strip.open()
#         self.socket = PowerUSBSocket(self.strip,1)
#         self.socket.power = 'off'
#
#     def toggleVaccum(self, turnOn):
#         """Toggles the vaccum power"""
#         if turnOn:
#             self.socket.power = 'on'
#         else:
#             self.socket.power = 'off'

class SimulateTrajectory:
    """A robot trajectory defined by the robot and list of milestones"""
    def __init__(self, robot, milestones, speed=1):
        # self.robot = robot
        self.milestones = None
        self.curr_index = None
        self.curr_milestone = None
        self.speed = speed
        self.complete = False
        # Process milestones
        assert(0 < self.speed <= 1)
        self.milestones['database'] = milestones
        self.milestones['robot'] = [convertConfigToRobot(mi) for mi in milestones]

    def start(self):
        """Sets the current milestone to the first in the list"""
        self.curr_index = 0
        self.curr_milestone = self.milestones['robot'][0]
        self.startTime = time()
        # for m in self.milestones['robot']:
        #     dt = m[0]
        #     q = m[1]['robot']
        #     self.robot.client.addMilestone(dt,q)

    def update(self):
        """Checks the current milestone in progress and updates, if necessary"""
        elapsedTime = time() - self.startTime
        if elapsedTime >= self.curr_milestone[0]:
            self.advanceMilestone()

    def advanceMilestone(self):
        """Advances to the next milestone, if applicable"""
        self.curr_index += 1
        if (self.curr_index < len(self.milestones['robot'])):
            self.curr_milestone = self.milestones['robot'][self.curr_index]
            # turnVaccumOn = (self.curr_milestone[1]['vaccum'] == 'on')
            # self.robot.toggleVaccum(turnVaccumOn)
        else:
            self.complete = True
            self.curr_index = None
            self.curr_milestone = None


class SimulateRobotController:
    """Trajectory execution for TX90."""
    def __init__(self, robot='left', milestones=None, speed=1., store=None):
        # Robot
        # self.robot = SimulateRobot(robot)
        if milestones:
            self.trajectory = Trajectory(milestones, speed)
        else:
            self.trajectory = None
            raise RuntimeError('No trajectory to run found.')
        self.freq = 10.
        # Database
        self.store = store or PensiveClient().default()

    def run(self):
        """Runs the current trajectory in the database"""
        # self.updatePlannedTrajectory()
        self.trajectory.start()
        self.loop()
        # print "current milestone", self.robot.getCurSegments()
        # print "milestones ",  self.robot.getRemainingSegments()
        # print "max segments ", self.robot.getMaxSegments()

    def loop(self):
        """Executed at the given frequency"""
        while not self.trajectory.complete:
            self.updateCurrentConfig()
            self.trajectory.update()
            sleep(1/float(self.freq))
        print "Trajectory completed"

    def updateCurrentConfig(self, path='robot/current_config'):
        """Updates the database with the robot's current configuration."""
        q = self.trajectory.curr_milestone
        q = convertConfigToDatabase(q) #TODO
        self.store.put(path, q)

    # def updatePlannedTrajectory(self, path='robot/waypoints'):
    #     """Updates the robot's planned trajectory from the database"""
    #     self.trajectory = Trajectory(self.robot, self.store.get(path))

if __name__ == "__main__":
    store = PensiveClient(host='http://10.10.1.102:8888/').default()
    c = SimulateRobotController(store=store)
    sample_milestones = [
           (2, {
              'robot': [0, 0, 0, 0, 0, 0, 0],
              'gripper': [0,0,0],
              'vaccum': [0]
            })
        ]
    store.put('robot/waypoints', sample_milestones)
    c.run()
