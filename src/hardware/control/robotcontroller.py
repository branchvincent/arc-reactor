"""Controller for sending trajectories to the TX90"""

# import sys; sys.path.append('../..')
import logging; logger = logging.getLogger(__name__)
from time import sleep
from copy import deepcopy
from math import degrees, radians, isnan, isinf
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum.powerusb.vpower import PowerUSBStrip, PowerUSBSocket

# List of robot IP addresses
robots = {  'left': '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

dof = 6
bufferSize = 100

# Helper functions

def Assert(condition, message):
    """Raises and logs an exception if the condition is false"""
    if not condition:
        logger.error(message)
        raise Exception(message)

def convertConfigToRobot(q_old, speed):
    """Converts database-style configuration to robot-stye configuration"""
    # Check for errors
    # Assert(q_old[1]['robot'])
    Assert(len(q_old[1]['robot']) == dof + 1, 'Configuration {} is of improper size'.format(q_old[1]['robot']))
    Assert(0 < speed <= 1, 'Speed {} is not in (0,1]'.format(speed))
    for qi in q[1]['robot']:
        Assert(not isinf(qi) and not isnan(qi), 'Invalid configuration {}'.format(qi))
    # Convert to robot
    q = list(deepcopy(q_old))
    q[0] /= float(speed)
    q[1]['robot'] = [degrees(qi) for qi in q[1]['robot']]
    q[1]['robot'][3] *= -1
    q[1]['robot'][5] *= -1
    del q[1]['robot'][0]
    return tuple(q)

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
    return tuple(q)

class Robot:
    """A robot defined by its trajectory client"""
    def __init__(self, robot='left', port=1000, store=None):
        Assert(robot in robots, 'Unrecognized robot {}'.format(robot))
        self.client = TrajClient(host=robots[robot], port=port)
        self.name = self.client.name
        self.store = store or PensiveClient().default()
        self.receivedMilestones = []
        # Vacuum
        self.vacuum = Vacuum()
        self.vacuum.on()
        # Queue
        self.startIndex = None

    def sendMilestone(self, milestone):
        """Sends a milestone to the robot"""
        dt = milestone[0]
        q = milestone[1]['robot']
        # Add milestone and update start index, if first milestone
        if not self.startIndex:
            self.startIndex = self.client.addMilestone(dt,q)
            Assert(self.startIndex, 'Could not add milestone {}'.format((dt,q)))
            logger.debug('Motion plan starting at index {}'.format(self.startIndex))
        # Add milestone without reply
        else:
            success = self.client.addMilestoneQuiet(dt,q)
            Assert(success, 'Could not add milestone {}'.format((dt,q)))
        self.receivedMilestones.append(milestone)
        indexRel = len(self.receivedMilestones) - 1
        indexAbs = self.startIndex + indexRel
        logger.info('Adding milestone {} or {}: {}'.format(indexRel,indexAbs,(dt,q)))

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

    def toggleVacuum(self, turnOn):
        """Toggles the vacuum power"""
        self.vacuum.change(turnOn)


class Trajectory:
    """A robot trajectory defined by the robot and list of milestones"""
    def __init__(self, robot, milestones, speed=1):
        self.robot = robot
        self.milestones = {}
        self.curr_index = None
        self.curr_milestone = None
        self.speed = speed
        self.complete = False
        # Process milestones
        Assert(0 < self.speed <= 1, 'Speed {} is not in (0,1]'.format(self.speed))
        self.milestones['database'] = milestones
        self.milestones['robot'] = [convertConfigToRobot(mi,self.speed) for mi in milestones]

    def start(self):
        """Sets the current milestone to the first in the list"""
        logger.info('Starting trajectory')
        self.curr_index = 0
        self.curr_milestone = self.milestones['robot'][self.curr_index]
        if (self.robot.getCurrentConfig() != self.curr_milestone[1]['robot']):    #TODO
            # Send additional config to robot
            m0 = deepcopy(self.curr_milestone[1])
            m0[0] = 2*self.speed
            m0[1]['robot'] = self.robot.getCurrentConfig()
            self.robot.sendMilestone(m0)
            logger.warning('Initial configuration is not current configuration.')
        for m in self.milestones['robot'][:bufferSize]:
            self.robot.sendMilestone(m)

    def update(self):
        """Checks the current milestone in progress and updates, if necessary"""
        actual_index = self.robot.getCurrentIndexRel()
        if self.curr_index != actual_index:
            self.advanceMilestone(actual_index)

    def advanceMilestone(self, milestone_index):
        """Advances to the next milestone, if applicable"""
        self.curr_index = milestone_index
        if self.curr_index < len(self.milestones['robot']):
            self.curr_milestone = self.milestones['robot'][self.curr_index]
            turnVacuumOn = (self.curr_milestone[1].get('vacuum', 'off') == 'on')
            self.robot.toggleVacuum(turnVacuumOn)
            if bufferSize + self.curr_index - 1 < len(self.milestones['robot']):
                self.robot.sendMilestone(self.milestones['robot'][bufferSize + self.curr_index - 1])
        else:
            self.complete = True
            self.curr_index = None
            self.curr_milestone = None
        logger.info('Moving to milestone {}'.format(self.robot.getCurrentIndexAbs()))


class RobotController:
    """Trajectory execution for TX90."""
    def __init__(self, robot='left', milestones=None, speed=1., store=None):
        # Robot
        self.robot = Robot(robot=robot, store=store)
        if milestones:
            self.trajectory = Trajectory(robot=self.robot, milestones=milestones, speed=speed)
        else:
            self.trajectory = None
        self.freq = 10.
        # Database
        self.store = store or PensiveClient().default()

    def run(self):
        """Runs the current trajectory in the database"""
        # self.updatePlannedTrajectory()
        if self.trajectory:
            self.trajectory.start()
            self.loop()
        else:
            logger.warning('No trajectory found')

    def loop(self):
        """Executed at the given frequency"""
        while not self.trajectory.complete:
            self.updateCurrentConfig()
            self.trajectory.update()
            sleep(1/float(self.freq))
        logger.info('Trajectory completed.')

    def updateCurrentConfig(self, path='robot/current_config'):
        """Updates the database with the robot's current configuration."""
        q = self.robot.getCurrentConfig()
        q = convertConfigToDatabase(q) #TODO
        self.store.put(path, q)

    # def updatePlannedTrajectory(self, path='robot/waypoints'):
    #     """Updates the robot's planned trajectory from the database"""
    #     self.trajectory = Trajectory(self.robot, self.store.get(path))

if __name__ == "__main__":
    store = PensiveClient(host='http://10.10.1.102:8888/').default()
    # c = RobotController(store=store)
    # sample_milestones = [
    #        (2, {
    #           'robot': [0, 0, 0, 0, 0, 0, 0],
    #           'gripper': [0,0,0],
    #           'vacuum': [0]
    #         })
    #     ]
    # store.put('robot/waypoints', sample_milestones)
    # c.run()
