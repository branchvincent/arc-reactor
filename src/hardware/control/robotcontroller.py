"""Controller for sending trajectories to the TX90"""

from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.gripper import Gripper
from hardware.vacuum import Vacuum
from motion.checker import MotionPlanChecker
from motion.milestone import Milestone
from master.world import build_world, klampt2numpy

from time import sleep
import logging; logger = logging.getLogger(__name__)

bufferSize = 100

# Helper functions
def Assert(condition, message):
    """Raises and logs an exception if the condition is false"""
    if not condition:
        logger.error(message)
        raise Exception(message)


class RobotController:
    """Trajectory execution for TX90"""
    def __init__(self, robot='left', store=None):
        self.store = store or PensiveClient().default()
        self.robot = Robot(robot=robot, store=self.store)
        self.trajectory = Trajectory(robot=self.robot, store=self.store)
        self.freq = 10.

    def run(self):
        """Runs the current trajectory in the database"""
        if self.trajectory is None or len(self.trajectory.milestones) == 0:
            logger.warn('Tried to execute an empty motion plan')
        else:
            # self.trajectory.check()
            self.trajectory.start()
            self.loop()
            self.updateDatabase()

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
        m = Milestone(robot=q, type='robot')
        m.set_type('db')
        self.store.put('/robot/current_config', m.get_robot())

    def updateDatabase(self):
        # Update tool pose
        world = build_world(self.store, ignore=['obstacles', 'camera', 'boxes', 'totes', 'shelf'])
        robot = world.robot('tx90l')
        T_tcp = klampt2numpy(robot.link(robot.numLinks() - 1).getTransform())
        self.store.put('/robot/tcp_pose', T_tcp)
        # Update tool camera pose
        T_cam = self.store.get('/robot/camera_xform')
        T = T_tcp.dot(T_cam)
        self.store.put('camera/tcp/pose', T)

    # def updatePlannedTrajectory(self, path='robot/waypoints'):
    #     """Updates the robot's planned trajectory from the database"""
    #     self.trajectory = Trajectory(self.robot, self.store.get(path))


class Trajectory:
    """A robot trajectory defined by the robot and list of milestones"""
    def __init__(self, robot, store=None):
        self.store = store or PensiveClient().default()
        self.robot = robot
        self.vacuum = Vacuum(store=self.store)
        self.gripper = Gripper(store=self.store)
        # Get milestone maps
        milestoneMaps = self.store.get('/robot/waypoints', [])
        speed = self.store.get('/robot/speed_scale', 1.)
        # Create milestones
        self.milestones = [Milestone(map=map) for map in milestoneMaps]
        for m in self.milestones:
            m.set_type('robot')
            m.scale_t(speed)
        self.reset()

    def reset(self):
        self.curr_index = None
        self.curr_milestone = None
        self.complete = False

    def check(self):
        """Sends milestones to motion plan checker"""
        q0 = self.robot.getCurrentConfig()
        c = MotionPlanChecker(self.milestones, q0=q0)
        failures = c.check()
        if failures:
            raise RuntimeError('Motion plan failed to pass checker')

    def start(self):
        """Sets the current milestone to the first in the list"""
        logger.info('Starting trajectory')
        self.curr_index = 0
        self.curr_milestone = self.milestones[self.curr_index]
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
            self.vacuum.change(bool(self.curr_milestone.get_vacuum()[0]))
            self.gripper.command(self.curr_milestone.get_gripper()[0])
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
    def __init__(self, robot='right', port=1000, store=None):
        self.store = store or PensiveClient().default()
        robots = self.store.get('/system/robots', {})
        Assert(robot in robots, 'Unrecognized robot {}'.format(robot))
        logger.warn('Connecting to {} at {}'.format(robot, robots[robot]))
        self.client = TrajClient(host=robots[robot], port=port)
        # self.name = self.client.name
        self.receivedMilestones = []
        self.startIndex = None

    def sendMilestone(self, milestone):
        """Sends a milestone to the robot"""
        Assert(isinstance(milestone, Milestone), 'Milestone must be instance of "Milestone"')
        Assert(milestone.get_type() == 'robot', 'Milestone must be of type "robot"')
        dt = milestone.get_t()
        q = milestone.get_robot()
        # Add milestone and update start index, if first milestone
        if self.startIndex == None:
            self.startIndex = 0
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
    # s = PensiveClient().default()
    c = RobotController()
    # c.run()
    # c.updateCurrentConfig()
