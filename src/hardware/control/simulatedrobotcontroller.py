"""Controller for simulating sending trajectories to the TX90"""

import logging; logger = logging.getLogger(__name__)
from time import sleep, time
from copy import deepcopy
from math import pi, degrees, radians
from math import degrees, radians, isnan, isinf, ceil
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum import Vacuum
import numpy as np

from master.world import build_world, klampt2numpy
from klampt.model.collide import WorldCollider
from klampt.plan.robotcspace import RobotCSpace

# List of robot IP addresses
# dof = 6
bufferSize = 100
robots = {  'left': '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

def Assert(condition, message):
    """Raises and logs an exception if the condition is false"""
    if not condition:
        logger.error(message)
        raise Exception(message)


class SimulatedRobotController:
    """Trajectory execution for TX90"""
    def __init__(self, robot='left', store=None):
        self.store = store or PensiveClient().default()
        # Milestones
        self.trajectory = SimulatedTrajectory(store=self.store)
        if self.trajectory.milestones is not None:
            self.freq = self.trajectory.milestones[0].get_t()
        else:
            self.freq = 60
        self.startTime = None

    def run(self):
        """Runs the current trajectory in the database"""
        self.startTime = time()
        self.trajectory.start()
        self.loop()
        self.updateDatabase()

    def loop(self):
        """Executed at the given frequency"""
        while not self.trajectory.complete:
            self.updateCurrentConfig()
            elapsedTime = time() - self.startTime
            self.trajectory.update(int(elapsedTime*self.freq))
            sleep(1/(3*self.freq))
        logger.info('Trajectory completed')

    def updateCurrentConfig(self):
        """Updates the database with the robot's current configuration."""
        m = self.trajectory.curr_milestone
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


class SimulatedTrajectory:
    """A robot trajectory defined by the robot and list of milestones"""
    def __init__(self, store=None):
        self.store = store or PensiveClient().default()
        self.vacuum = Vacuum(store=self.store)
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
        self.curr_index = 0
        self.curr_milestone = self.milestones[self.curr_index]
        logger.info('Starting trajectory')
        # dt = self.curr_milestone.get_t()
        # q = [round(qi,1) for qi in self.curr_milestone.get_robot()]
        # logger.info('Moving to milestone {}: {}'.format(self.curr_index, (round(dt,3),q)))

    def update(self, index):
        """Checks the current milestone in progress and updates, if necessary"""
        if self.curr_index != index:
            self.advanceMilestone(index)

    def advanceMilestone(self, index):
        """Advances to the next milestone, if applicable"""
        self.curr_index = index
        if self.curr_index < len(self.milestones):
            self.curr_milestone = self.milestones[self.curr_index]
            self.vacuum.change(self.curr_milestone.get_vacuum())
            # dt = self.curr_milestone.get_t()
            # qf = [round(qi,1) for qi in self.curr_milestone[1]['robot']]
            # logger.info('Moving to milestone {}: {}'.format(self.curr_index, (round(dt,3),qf)))
        else:
            self.reset()
            self.complete = True


if __name__ == "__main__":
    store = PensiveClient().default()
    # c = SimulatedRobotController(store=store)
    # c.jogTo([0]*7)
