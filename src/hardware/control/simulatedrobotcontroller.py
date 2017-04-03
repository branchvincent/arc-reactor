# For executing trajectories

# import sys; sys.path.append('../..')
import logging; logger = logging.getLogger(__name__)
from time import sleep, time
from copy import deepcopy
from math import pi, degrees, radians
from math import degrees, radians, isnan, isinf, ceil
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum import Vacuum
import numpy as np

from master.world import build_world
from klampt.model.collide import WorldCollider
from klampt.plan.robotcspace import RobotCSpace

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

def createMilestoneMap(dt,qs,type='db',to='robot'): #degs=False,
    Assert(type in ['robot','db'], 'Unrecognized option: "{}"'.format(type))
    Assert(to in ['robot','db'], 'Unrecognized option: "{}"'.format(to))
    if type == 'db' and to == 'robot':
        qs = [[degrees(qi) for qi in q] for q in qs]
        qs = [q[1:] for q in qs]
        for q in qs:
            q[2] *= -1
            q[4] *= -1
    elif type =='robot' and to == 'db':
        qs = [[radians(qi) for qi in q] for q in qs]
        qs = [[0] + q for q in qs]
        for q in qs:
            q[3] *= -1
            q[5] *= -1
    elif type == to:
        pass
    # if type == 'db':
    #     if degs:
    #         qs = [[radians(qi) for qi in q] for q in qs]
    #     if len(qs) > 0 and len(qs[0]) == dof:
    #         qs = [[0] + q for q in qs]
    # elif type == 'robot':
    #     if not degs:
    #         qs = [[degrees(qi) for qi in q] for q in qs]
    #     if len(qs) > 0 and len(qs[0]) == dof + 1:
    #         qs = [q[1:] for q in qs]
    return [[dt,{'robot': qi}] for qi in qs]

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

    def jogTo(self,qdes,rads=True):
        """Jogs the robot to the specified configuration, in radians"""
        # Setup world and cspace
        world = build_world(self.store)
        robotSim = world.robot('tx90l')
        collider = WorldCollider(world)
        cspace = RobotCSpace(robotSim,collider=collider)

        # Convert configuration
        if not rads:
            qdes = [radians(qi) for qi in qdes]
        q0 = self.store.get('robot/current_config')

        # Determine discretization
        dq_max, dv_max = radians(5.), radians(60.)
        distance = robotSim.distance(q0,qdes)
        numMilestones = 1 + ceil(distance/dq_max)
        dt = 1.5 * dq_max/dv_max
        logger.info("Jogging from {} to {}, with {} milestones".format(q0,qdes,numMilestones))

        # Create milestones, checking for feasibility
        qs = [robotSim.interpolate(q0,qdes,ui) for ui in np.linspace(0,1,numMilestones)]
        feasible = True
        for qi in qs:
            if not cspace.inJointLimits(qi):
                logger.warn("Configuration not in joint limits")
                feasible = False
                break
            elif cspace.selfCollision(x=qi):
                logger.warn("Configuration colliding with self")
                feasible = False
                break
            elif cspace.envCollision(x=qi):
                logger.warn("Configuration colliding with environment")
                feasible = False
                break

        # Run path, if feasible
        if feasible:
            milestones = createMilestoneMap(dt,qs,type='db',to='db')
            self.trajectory = SimulatedTrajectory(milestones=milestones)
            self.run()
        else:
            logger.warn("Jogger could not find feasible path")


if __name__ == "__main__":
    store = PensiveClient().default()
    c = SimulatedRobotController(store=store)
    m = [0]*7
    c.jogTo(m)
