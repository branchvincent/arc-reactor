# For executing trajectories

# import sys; sys.path.append('../..')

import logging; logger = logging.getLogger(__name__)
from time import sleep, time
from copy import deepcopy
from math import pi, degrees, radians
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum.powerusb.vpower import PowerUSBStrip, PowerUSBSocket

# List of robot IP addresses
robots = {  'left': '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

def convertConfigToRobot(q_old, speed):
    assert(len(q_old[1]['robot']) == 7)
    assert(0 < speed <= 1)
    #print "From ", q_old
    q = list(deepcopy(q_old))
    q[0] /= float(speed)
    q[1]['robot'] = [degrees(qi) for qi in q[1]['robot']]
    q[1]['robot'][3] *= -1
    q[1]['robot'][5] *= -1
    del q[1]['robot'][0]
    #print "To ", q
    return tuple(q)

def convertConfigToDatabase(q_old):
    assert(len(q_old) == 6)
    q = list(deepcopy(q_old))
    q = [radians(qi) for qi in q]
    q.insert(0,0)
    q[3] *= -1
    q[5] *= -1
    return tuple(q)

# class SimulatedRobot:
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
#     def toggleVacuum(self, turnOn):
#         """Toggles the vacuum power"""
#         if turnOn:
#             self.socket.power = 'on'
#         else:
#             self.socket.power = 'off'

class SimulatedTrajectory:
    """A robot trajectory defined by the robot and list of milestones"""
    def __init__(self, milestones, speed=1, store=None):
        # self.robot = robot
        self.milestones = {}
        self.curr_index = None
        self.curr_milestone = None
        self.speed = speed
        self.complete = False
        # Process milestones
        assert(0 < self.speed <= 1)
        self.milestones['database'] = milestones
        self.milestones['robot'] = [convertConfigToRobot(mi,speed) for mi in milestones]
        # store
        if store:
            self.store = store
        else:
            self.store = PensiveClient().default()

    def start(self):
        """Sets the current milestone to the first in the list"""
        self.curr_index = 0
        self.curr_milestone = self.milestones['robot'][self.curr_index]
        self.startTime = time()
        print "Starting traj at ", self.startTime

    def update(self):
        """Checks the current milestone in progress and updates, if necessary"""
        self.advanceMilestone()

    def advanceMilestone(self):
        """Advances to the next milestone, if applicable"""
        self.curr_index += 1
        if (self.curr_index < len(self.milestones['robot'])):
            self.curr_milestone = self.milestones['robot'][self.curr_index]
            turnVacuumOn = (self.curr_milestone[1]['vacuum'] == 'on')
            self.store.put('/vacuum/power', turnVacuumOn)
        else:
            self.complete = True
            self.curr_index = None
            self.curr_milestone = None
        #print "Advancing to milestone ", self.curr_index, " after ", time() - self.startTime, " s"
        self.startTime = time()


class SimulatedRobotController:
    """Trajectory execution for TX90."""
    def __init__(self, robot='left', milestones=None, speed=1., store=None):
        # Robot
        # self.robot = SimulatedRobot(robot)
        if milestones:
            self.trajectory = SimulatedTrajectory(milestones=milestones, speed=speed, store=store)
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

    def loop(self):
        """Executed at the given frequency"""
        while not self.trajectory.complete:
            #print "Starting loop..."
            self.updateCurrentConfig()
            napTime = self.trajectory.curr_milestone[0] - (time() - self.trajectory.startTime)
            if napTime > 0:
                sleep(napTime)
            self.trajectory.update()

        print "Trajectory completed"

    def updateCurrentConfig(self, path='robot/current_config'):
        """Updates the database with the robot's current configuration."""
        q = self.trajectory.curr_milestone
        q = convertConfigToDatabase(q[1]['robot'])
        self.store.put(path, q)

if __name__ == "__main__":
    store = PensiveClient().default()
    c = SimulatedRobotController(store=store)
    # store.put('robot/waypoints', sample_milestones)
    # c.run()
