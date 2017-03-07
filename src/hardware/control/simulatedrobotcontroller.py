# For executing trajectories

# import sys; sys.path.append('../..')
import logging; logger = logging.getLogger(__name__)
from time import sleep, time
from copy import deepcopy
from math import pi, degrees, radians
from math import degrees, radians, isnan, isinf
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum import Vacuum

# List of robot IP addresses
dof = 6
bufferSize = 100
robots = {  'left': '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

def Assert(condition, message):
    """Raises and logs an exception if the condition is false"""
    if not condition:
        logger.error(message)
        raise Exception(message)

def convertConfigToRobot(q_old, speed):
    """Converts database-style configuration to robot-stye configuration"""
    # Check for errors
    Assert(len(q_old[1]['robot']) == dof + 1, 'Configuration {} is of improper size'.format(q_old[1]['robot']))
    Assert(0 < speed <= 1, 'Speed {} is not in (0,1]'.format(speed))
    for qi in q_old[1]['robot']:
        Assert(not isinf(qi) and not isnan(qi), 'Invalid configuration {}'.format(qi))
    # Convert to robot
    q = list(deepcopy(q_old))
    q[0] /= float(speed)
    q[1]['robot'] = [degrees(qi) for qi in q[1]['robot']]
    q[1]['robot'][3] *= -1
    q[1]['robot'][5] *= -1
    del q[1]['robot'][0]
    return q

def convertConfigToDatabase(q_old):
    """Converts robot-style configuration to database-stye configuration"""
    # Check for errors
    Assert(len(q_old) == dof, 'Configuration {} is of improper size'.format(q_old))
    for qi in q_old:
        Assert(not isinf(qi) and not isnan(qi), 'Invalid configuration {}'.format(qi))
    # Convert to database
    q = list(deepcopy(q_old))
    q = [radians(qi) for qi in q]
    q.insert(0,0)
    q[3] *= -1
    q[5] *= -1
    return q


class SimulatedTrajectory:
    """A robot trajectory defined by the robot and list of milestones"""
    def __init__(self, milestones, speed=1, store=None):
        self.milestones = {}
        self.curr_index = None
        self.curr_milestone = None
        self.speed = speed
        self.complete = False
        self.vacuum = Vacuum(store=store)
        # Process milestones
        Assert(0 < self.speed <= 1, 'Speed {} is not in (0,1]'.format(self.speed))
        self.milestones['database'] = milestones
        self.milestones['robot'] = [convertConfigToRobot(mi,speed) for mi in milestones]
        # store
        self.store = store or PensiveClient().default()

    def start(self):
        """Sets the current milestone to the first in the list"""
        self.curr_index = 0
        self.curr_milestone = self.milestones['robot'][self.curr_index]
        logger.info('Starting trajectory')
        dt = self.curr_milestone[0]
        q = [round(qi,1) for qi in self.curr_milestone[1]['robot']]
        logger.info('Moving to milestone {}: {}'.format(self.curr_index, (round(dt,3),q)))

    def update(self, index):
        """Checks the current milestone in progress and updates, if necessary"""
        if self.curr_index != index:
            self.advanceMilestone(index)

    def advanceMilestone(self, index):
        """Advances to the next milestone, if applicable"""
        self.curr_index = index
        if self.curr_index < len(self.milestones['robot']):
            self.curr_milestone = self.milestones['robot'][self.curr_index]
            self.vacuum.change(self.curr_milestone[1].get('vacuum', [0])[0] > 0)
            dt = self.curr_milestone[0]
            qf = [round(qi,1) for qi in self.curr_milestone[1]['robot']]
            logger.info('Moving to milestone {}: {}'.format(self.curr_index, (round(dt,3),qf)))
        else:
            self.complete = True
            self.curr_index = None
            self.curr_milestone = None


class SimulatedRobotController:
    """Trajectory execution for TX90."""
    def __init__(self, robot='left', milestones=None, speed=1., store=None):
        # Robot
        if milestones:
            self.trajectory = SimulatedTrajectory(milestones=milestones, speed=speed, store=store)
            self.freq = 1/milestones[0][0]
        else:
            self.trajectory = None
            self.freq = 60
        self.startTime = None
        # Database
        self.store = store or PensiveClient().default()

    def run(self):
        """Runs the current trajectory in the database"""
        self.startTime = time()
        self.trajectory.start()
        self.startTime = time()
        self.loop()

    def loop(self):
        """Executed at the given frequency"""
        while not self.trajectory.complete:
            self.updateCurrentConfig()
            elapsedTime = time() - self.startTime
            self.trajectory.update(int(elapsedTime*self.freq))
            sleep(1/(3*self.freq))
        logger.info('Trajectory completed')

    def updateCurrentConfig(self, path='robot/current_config'):
        """Updates the database with the robot's current configuration."""
        q = self.trajectory.curr_milestone
        q = convertConfigToDatabase(q[1]['robot'])
        self.store.put(path, q)

# if __name__ == "__main__":
#     store = PensiveClient().default()
#     c = SimulatedRobotController(store=store)
    # store.put('robot/waypoints', sample_milestones)
    # c.run()
