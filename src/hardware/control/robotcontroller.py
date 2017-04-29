"""Controller for sending trajectories to the TX90"""

from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum import Vacuum
from motion.checker import MotionPlanChecker
from master.world import build_world, klampt2numpy
from motion.milestone import Milestone

from klampt.model.collide import WorldCollider
from klampt.plan.robotcspace import RobotCSpace
from klampt.math import vectorops

import numpy as np
from time import time
from time import sleep
from copy import deepcopy
from math import degrees, radians, isnan, isinf, ceil

import logging; logger = logging.getLogger(__name__)

# List of robot IP addresses
dof = 6
bufferSize = 100
robots = {  'left':  '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

# Helper functions

def Assert(condition, message):
    """Raises and logs an exception if the condition is false"""
    if not condition:
        logger.error(message)
        raise Exception(message)

# def createMilestoneMap(dt,qs,type='db',to='robot'): #degs=False,
#     Assert(type in ['robot','db'], 'Unrecognized option: "{}"'.format(type))
#     Assert(to in ['robot','db'], 'Unrecognized option: "{}"'.format(to))
#     if type == 'db' and to == 'robot':
#         qs = [[degrees(qi) for qi in q] for q in qs]
#         qs = [q[1:] for q in qs]
#         for q in qs:
#             q[2] *= -1
#             q[4] *= -1
#     elif type =='robot' and to == 'db':
#         qs = [[radians(qi) for qi in q] for q in qs]
#         qs = [[0] + q for q in qs]
#         for q in qs:
#             q[3] *= -1
#             q[5] *= -1
#     elif type == to:
#         pass
#     # if type == 'db':
#     #     if degs:
#     #         qs = [[radians(qi) for qi in q] for q in qs]
#     #     if len(qs) > 0 and len(qs[0]) == dof:
#     #         qs = [[0] + q for q in qs]
#     # elif type == 'robot':
#     #     if not degs:
#     #         qs = [[degrees(qi) for qi in q] for q in qs]
#     #     if len(qs) > 0 and len(qs[0]) == dof + 1:
#     #         qs = [q[1:] for q in qs]
#     return [[dt,{'robot': qi}] for qi in qs]

# def convertConfigToRobot(q_old, speed):
#     """Converts database-style configuration to robot-stye configuration"""
#     # Check for errors
#     Assert(len(q_old[1]['robot']) == dof + 1, 'Configuration {} is of improper size'.format(q_old[1]['robot']))
#     Assert(0 < speed <= 1, 'Speed {} is not in (0,1]'.format(speed))
#     for qi in q_old[1]['robot']:
#         Assert(not isinf(qi) and not isnan(qi), 'Invalid configuration {}'.format(qi))
#     # Convert to robot
#     q = list(deepcopy(q_old))
#     q[0] /= float(speed)
#     q[1]['robot'] = [degrees(qi) for qi in q[1]['robot']]
#     q[1]['robot'][3] *= -1
#     q[1]['robot'][5] *= -1
#     del q[1]['robot'][0]
#     return q
#
# def convertConfigToDatabase(q_old):
#     """Converts robot-style configuration to database-stye configuration"""
#     # Check for errors
#     Assert(len(q_old) == dof, 'Configuration {} is of improper size'.format(q_old))
#     for qi in q_old:
#         Assert(not isinf(qi) and not isnan(qi), 'Invalid configuration {}'.format(qi))
#     # Convert to database
#     q = list(deepcopy(q_old))
#     q = [radians(qi) for qi in q]
#     q.insert(0,0)
#     q[3] *= -1
#     q[5] *= -1
#     return q


class RobotController:
    """Trajectory execution for TX90"""
    def __init__(self, robot='left', store=None):
        self.store = store or PensiveClient().default()
        # Robot
        self.robot = Robot(robot=robot, store=self.store)
        # Milestones
        if self.milestones:
            self.trajectory = Trajectory(robot=self.robot, store=self.store)
        else:
            self.trajectory = None
        self.freq = 10.

    def run(self):
        """Runs the current trajectory in the database"""
        if self.trajectory:
            self.trajectory.start()
            self.loop()
            self.updateDatabase()
        else:
            logger.warning('No trajectory found')

    def loop(self):
        """Executed at the given frequency"""
        while not self.trajectory.complete:
            self.updateCurrentConfig()
            self.trajectory.update()
            sleep(1/float(self.freq))
        logger.info('Trajectory completed.')

    def updateCurrentConfig(self):
        """Updates the database with the robot's current configuration."""
        q = self.robot.getCurrentConfig()
        m = Milestone(dt=0, q=q, type='robot')
        m.set_type('db')
        self.store.put('/robot/current_config', m.get_robot())

    def updateDatabase(self):
        # Update tool pose
        world = build_world(self.store)
        robot = world.robot('tx90l')
        T_tcp = klampt2numpy(robot.link(robot.numLinks() - 1).getTransform())
        self.store.put('/robot/tcp_pose', T_tcp)
        # Update tool camera pose
        T_cam = self.store.get('/robot/camera_xform')
        T = T_tcp.dot(T_cam)
        self.store.put('camera/tcp/pose', T)

    # def jogTo(self,qdes,rads=True):
    #     """Jogs the robot to the specified configuration, in radians"""
    #     # NOTE: mimics PlanRoute state for now, by generating a motion plan...
    #
    #     # Setup world and cspace
    #     world = build_world(self.store)
    #     robotSim = world.robot('tx90l')
    #     collider = WorldCollider(world)
    #     cspace = RobotCSpace(robotSim,collider=collider)
    #
    #     # Convert configuration
    #     if not rads:
    #         qdes = [radians(qi) for qi in qdes]
    #     q0_robot = self.robot.getCurrentConfig()
    #     q0 = [0] + [radians(qi) for qi in q0_robot]
    #     q0[3] *= -1; q0[5] *= -1
    #
    #     # Determine discretization
    #     dq_max, dv_max = radians(5.), radians(60.)
    #     distance = robotSim.distance(q0,qdes)
    #     numMilestones = 1 + ceil(distance/dq_max)
    #     dt = 3 * dq_max/dv_max
    #     logger.info("Jogging from {} to {}, with {} milestones".format(q0,qdes,numMilestones))
    #
    #     # Create milestones, checking for feasibility
    #     qs = [robotSim.interpolate(q0,qdes,ui) for ui in np.linspace(0,1,numMilestones)]
    #     feasible = True
    #     for qi in qs:
    #         if not cspace.inJointLimits(qi):
    #             logger.warn("Configuration not in joint limits")
    #             feasible = False
    #             break
    #         elif cspace.selfCollision(x=qi):
    #             logger.warn("Configuration colliding with self")
    #             feasible = False
    #             break
    #         elif cspace.envCollision(x=qi):
    #             logger.warn("Configuration colliding with environment")
    #             feasible = False
    #             break
    #
    #     # Run path, if feasible
    #     if feasible:
    #         milestones = createMilestoneMap(dt,qs,type='db',to='db')
    #         self.store.put('/robot/waypoints', milestones)
    #         self.store.put('/status/route_plan', True)
    #         self.store.put('/robot/timestamp', time())
    #         # self.trajectory = Trajectory(robot=self.robot, milestones=milestones, speed=1)
    #         # self.run()
    #         # self.store.put('robot/jog_config', qdes)
    #     else:
    #         self.store.put('/status/route_plan', False)
    #         logger.warn("Jogger could not find feasible path")
    #     return feasible

    # def updatePlannedTrajectory(self, path='robot/waypoints'):
    #     """Updates the robot's planned trajectory from the database"""
    #     self.trajectory = Trajectory(self.robot, self.store.get(path))


class Trajectory:
    """A robot trajectory defined by the robot and list of milestones"""
    def __init__(self, robot, store=None):
        self.store = store or PensiveClient().default()
        self.robot = robot
        # Get milestone maps
        milestoneMaps = self.store.get('/robot/waypoints')
        speed = self.store.get('/robot/speed_scale', 1.)
        # Create milestones
        self.milestones = []
        for map in milestoneMaps:
            m = Milestone(map=map)
            m.set_type('robot')
            m.scale_t(speed)
            self.milestones.append(m)
        # Validate
        self.check()
        self.reset()

    def reset(self):
        self.curr_index = None
        self.curr_milestone = None
        self.complete = False

    def check(self):
        """Sends milestones to motion plan checker"""
        c = MotionPlanChecker(self.milestones)
        failures = c.check()
        if failures:
            raise RuntimeError('Motion plan failed to pass checker')

    def start(self):
        """Sets the current milestone to the first in the list"""
        logger.info('Starting trajectory')
        self.curr_index = 0
        self.curr_milestone = self.milestones[self.curr_index]
        # Check initial config is current config
        # if (self.robot.getCurrentConfig() != self.curr_milestone[1]['robot']):
        #     self.curr_milestone[0] = 3
        #     logger.warning('Initial configuration is not current configuration.')
        # HACK: Delay first milestone to create sufficient buffering
        self.curr_milestone.set_t(5)
        # Send first milestones
        for m in self.milestones[:bufferSize]:
            self.robot.sendMilestone(m)

    def update(self):
        """Checks the current milestone in progress and updates, if necessary"""
        actual_index = self.robot.getCurrentIndexRel()
        # Update milestone, if current belief is wrong
        if self.curr_index != actual_index:
            self.advanceMilestone(actual_index)

    def advanceMilestone(self, milestone_index):
        """Advances to the next milestone, if applicable"""
        missed_milestones = milestone_index - self.curr_index
        self.curr_index = milestone_index
        # Update milestone
        if self.curr_index < len(self.milestones):
            self.curr_milestone = self.milestones[self.curr_index]
            self.robot.vacuum.change(self.curr_milestone.get_vacuum())
            logger.info('Moving to milestone {}'.format(self.robot.getCurrentIndexAbs()))
            # Add new milestone
            if len(self.robot.receivedMilestones) < len(self.milestones):
                start = len(self.robot.receivedMilestones)
                stop = start + missed_milestones
                for m in self.milestones[start:stop]:
                    self.robot.sendMilestone(m)
        # Trajectory finished
        else:
            self.reset()
            self.complete = True

class Robot:
    """A robot defined by its trajectory client"""
    def __init__(self, robot='left', port=1000, store=None):
        Assert(robot in robots, 'Unrecognized robot {}'.format(robot))
        self.client = TrajClient(host=robots[robot], port=port)
        # self.name = self.client.name
        self.store = store or PensiveClient().default()
        self.receivedMilestones = []
        # Vacuum
        self.vacuum = Vacuum(store=self.store)
        # Queue
        self.startIndex = None

    def sendMilestone(self, milestone):
        """Sends a milestone to the robot"""
        Assert(isinstance(milestone, Milestone), 'Milestone must be instance of "Milestone"')
        dt = milestone.get_t()
        q = milestone.get_robot()
        # Add milestone and update start index, if first milestone
        if self.startIndex == None:
            self.startIndex = self.client.addMilestone(dt,q)
            Assert(self.startIndex != None, 'Could not add milestone {}'.format((dt,q)))
            logger.debug('Motion plan starting at index {}'.format(self.startIndex))
        # Add milestone without reply
        else:
            self.client.addMilestoneQuiet(dt,q)
        self.receivedMilestones.append(milestone)
        # indexRel = len(self.receivedMilestones) - 1
        # indexAbs = self.startIndex + indexRel
        # qf = [round(qi,1) for qi in q]
        # logger.info('Adding milestone {}, {}: {}'.format(indexRel,indexAbs,(round(dt,3),qf)))

    def getCurrentMilestone(self):
        """Returns the milestone currently being executed"""
        index = self.getCurrentIndexRel()
        return self.receivedMilestones[index]

    def getCurrentIndexRel(self):
        """Returns the current milestone index, relative to the current motion plan"""
        return len(self.receivedMilestones) - self.client.getCurSegments()

    def getCurrentIndexAbs(self):
        """Returns the current milestone index, relative to the trajclient"""
        return self.startIndex + self.getCurrentIndexRel()

    def getCurrentConfig(self):
        """Returns the robot's current configuration"""
        return self.client.getConfig()


if __name__ == "__main__":
    s = PensiveClient().default()
    milestones = s.get('robot/waypoints')
    # c = RobotController(milestones=milestones)
    # c.run()
    # c.updateCurrentConfig()
