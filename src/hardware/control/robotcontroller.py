# For executing trajectories
import sys; sys.path.append('../..')
import logging; logger = logging.getLogger(__name__)
from time import sleep
from copy import deepcopy
from math import pi, degrees, radians
from pensive.client import PensiveClient
from hardware.tx90l.trajclient.trajclient import TrajClient
from hardware.vacuum.vpower import PowerUSBStrip, PowerUSBSocket

# List of robot IP addresses
robots = {  'left': '10.10.1.202',
            'right': '10.10.1.203',
            'local': 'localhost'    }

bufferSize = 100

# if len(q) != self.robot.dof:
#     logger.error('Incorrect robot configuration length.')
#     raise Exception('Incorrect robot configuration length.')

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
    # assert(len(q_old[1]['robot']) == 6)
    # q = list(deepcopy(q_old))
    # q[1]['robot'].insert(0,0)
    # q[1]['robot'] = [radians(qi) for qi in q[1]['robot']]
    # q[1]['robot'][3] *= -1
    # q[1]['robot'][5] *= -1
    # return tuple(q)
    assert(len(q_old) == 6)
    q = list(deepcopy(q_old))
    q = [radians(qi) for qi in q]
    q.insert(0,0)
    q[3] *= -1
    q[5] *= -1
    return tuple(q)

class Robot:
    """A robot defined by its trajectory client"""
    def __init__(self, robot='left', port=1000, store=None):
        assert(robot in robots)
        self.client = TrajClient(host=robots[robot], port=port)
        self.name = self.client.name
        self.dof = 6
        self.store = store or PensiveClient().default()
        self.receivedMilestones = {}
        # USB
        if not self.store.get('/simulate/vacuum'):
            print "accessing vacuum"
            self.strips = PowerUSBStrip().strips()
            self.strip = self.strips[0]
            self.socket = PowerUSBSocket(self.strip,1)
            self.strip.open()
            self.socket.power = "off"
        else:
            self.strip = None
            self.socket = None

        # Queue
        self.startIndex = None

    def sendMilestone(self, milestone):
        dt = milestone[0]
        q = milestone[1]['robot']
        absIndex = self.client.addMilestone(dt,q)
        self.receivedMilestones[absIndex] = milestone
        if not self.startIndex:
            self.startIndex = absIndex
            #print "Updating start index to ", self.startIndex
        elif absIndex < self.startIndex:
            self.startIndex = absIndex
            #print "Err: Updating start index to ", self.startIndex
        #print 'Adding milestone', q, 'at dt', dt, ' index ', absIndex

    def getCurrentMilestone(self):
        index = self.getCurrentIndexAbs()
        return self.receivedMilestones[index]

    def getCurrentIndexRel(self):
        return len(self.receivedMilestones) - self.client.getCurSegments()

    def getCurrentIndexAbs(self):
        return self.startIndex + self.getCurrentIndexRel()

    def toggleVacuum(self, turnOn):
        """Toggles the vacuum power"""
        if self.socket:
            print "have a socket"
            self.strip.open()
            if turnOn:
                print "turning on"
                self.socket.power = "on"
            else:
                print "turning off"
                self.socket.power = "off"


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
        assert(0 < self.speed <= 1)
        self.milestones['database'] = milestones
        self.milestones['robot'] = [convertConfigToRobot(mi,self.speed) for mi in milestones]

    def start(self):
        """Sets the current milestone to the first in the list"""
        print "Entering start..."
        self.curr_index = 0
        self.curr_milestone = self.milestones['robot'][self.curr_index]
        for m in self.milestones['robot'][:bufferSize]:
            self.robot.sendMilestone(m)
        print "Exiting start..."

    def update(self):
        """Checks the current milestone in progress and updates, if necessary"""
        if self.curr_index != self.robot.getCurrentIndexRel():
            self.advanceMilestone()

    def advanceMilestone(self):
        """Advances to the next milestone, if applicable"""
        self.curr_index += 1
        if self.curr_index < len(self.milestones['robot']):
            self.curr_milestone = self.milestones['robot'][self.curr_index]
            turnVacuumOn = (self.curr_milestone[1].get('vacuum', 'off') == 'on')
            print "turnvacuum on ", turnVacuumOn
            print "get ", self.curr_milestone[1].get('vacuum', 'off')
            self.robot.toggleVacuum(turnVacuumOn)
            if bufferSize + self.curr_index - 1 < len(self.milestones['robot']):
                self.robot.sendMilestone(self.milestones['robot'][bufferSize + self.curr_index - 1])
        else:
            self.complete = True
            self.curr_index = None
            self.curr_milestone = None
        print "Moving to milestone ", self.robot.getCurrentIndexAbs()


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
        self.trajectory.start()
        self.loop()

    def loop(self):
        """Executed at the given frequency"""
        while not self.trajectory.complete:
            self.updateCurrentConfig()
            self.trajectory.update()
            sleep(1/float(self.freq))
        print "Trajectory completed"

    def updateCurrentConfig(self, path='robot/current_config'):
        """Updates the database with the robot's current configuration."""
        q = self.robot.client.getConfig()
        q = convertConfigToDatabase(q) #TODO
        self.store.put(path, q)

    # def updatePlannedTrajectory(self, path='robot/waypoints'):
    #     """Updates the robot's planned trajectory from the database"""
    #     self.trajectory = Trajectory(self.robot, self.store.get(path))

if __name__ == "__main__":
    store = PensiveClient(host='http://10.10.1.102:8888/').default()
    c = RobotController(store=store)
    sample_milestones = [
           (2, {
              'robot': [0, 0, 0, 0, 0, 0, 0],
              'gripper': [0,0,0],
              'vacuum': [0]
            })
        ]
    store.put('robot/waypoints', sample_milestones)
    # c.run()
