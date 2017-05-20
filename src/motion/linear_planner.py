from pensive.client import PensiveClient
from master.world import build_world, numpy2klampt
from util.sync_robot import sync
from motion.milestone import Milestone

from klampt.model import ik
from klampt.math import vectorops as vops
from klampt.model.collide import WorldCollider
from klampt.plan.robotcspace import RobotCSpace

import copy
import time, math, numpy as np
import logging; logger = logging.getLogger(__name__)

class LinearPlanner:
    def __init__(self, store=None):
        self.store = store or PensiveClient().default()
        self.reset()

    def reset(self):
        self.world = build_world(self.store)
        self.robot = self.world.robot('tx90l')
        self.collider = WorldCollider(self.world)
        self.cspace = RobotCSpace(self.robot, collider=self.collider)

    def isFeasible(self, q):
        if not self.cspace.inJointLimits(q):
            logger.warn("Configuration not in joint limits")
            return False
        elif self.cspace.selfCollision(x=q):
            logger.warn("Configuration colliding with self")
            return False
        elif self.cspace.envCollision(x=q):
            logger.warn("Configuration colliding with environment")
            return False
        return True

    def getDesiredConfig(self, T=None, p=None):
        """Plans desired configuration to reach the specified ee transform"""
        ee_link = self.robot.link(6)

        if T is not None:
            if isinstance(T, np.ndarray):
                T = numpy2klampt(T)
            goal = ik.objective(ee_link, R=T[0], t=T[1])
        elif p is not None:
            goal = ik.objective(ee_link, local=p[0], world=p[1])
        else:
            raise RuntimeError('Must specify either p or T')

        if ik.solve(goal):
            return self.robot.getConfig()
        else:
            logger.warn('Could not find feasible configuration')
            raise RuntimeError('Could not find feasible configuration')
            # return self.robot.getConfig()

    def interpolate(self, q=None, T=None, p=None): #, rads=True):
        """Jogs the robot to the specified configuration, in radians"""
        # Get initial configuration
        q0 = self.store.get('/robot/current_config')
        if T is not None:
            q = self.getDesiredConfig(T=T)
        elif p is not None:
            q = self.getDesiredConfig(p=p)
        elif q is not None:
            raise RuntimeError('Must specify either p, q, or T')
        # if not rads:
        #     q = [math.radians(qi) for qi in q]

        # Get milestones and check feasibility
        feasible = True
        milestones = TimeScale(self.robot).getMilestones(q0, q)
        for m in milestones:
            feasible = self.isFeasible(m.get_robot())
            if not feasible:
                break

        # Update database
        if feasible:
            milestoneMap = [m.get_milestone() for m in milestones]
            self.store.put('/robot/waypoints', milestoneMap)
            self.store.put('/status/route_plan', True)
            self.store.put('/robot/timestamp', time.time())
        else:
            self.store.put('/status/route_plan', False)
            logger.warn('Could not find feasible path')


class TimeScale:
    def __init__(self, robot, type='linear', freq=20):
        self.robot = robot
        self.vmax = robot.getVelocityLimits()
        self.amax = robot.getAccelerationLimits()
        #HACK: incorrect a,v limits?
        k = 8
        self.vmax = [k*math.radians(vi) for vi in self.vmax]
        self.amax = [k*math.radians(ai) for ai in self.amax]
        self.type = type
        self.freq = freq
        if self.type not in ['linear', 'cubic']:
            raise RuntimeError('Type must be either linear or cubic')

    def getTimeScale(self, q0, qf):
        tf = []
        for q0i,qfi,vi,ai in zip(q0,qf,self.vmax,self.amax):
            D = abs(qfi - q0i)
            if self.type == 'linear':
                tf.append(D/vi if vi != 0 else 0)
            elif self.type == 'cubic':
                tf.append(max(1.5*D/vi, math.sqrt(6*D/ai)) if vi and ai != 0 else 0)
        return max(tf)

    def interpolate(self, q0, qf, t, tf):
        D = vops.sub(qf,q0)
        if self.type == 'linear':
            r = t/tf
        elif self.type == 'cubic':
            r = 3*(t/tf)**2 - 2*(t/tf)**3
        return vops.add(q0, vops.mul(D, t/tf))

    def getMilestones(self, q0, qf):
        milestones = []
        # Calculate duration
        tf = self.getTimeScale(q0,qf)
        # print 'Tf before', tf
        dt = 1/float(self.freq)
        if tf != 0:
            tf += abs(tf % dt - dt)     # make multiple of dt
            tf = max(tf, 1)             # enforce min time
        # print 'Tf after', tf
        # Append milestones
        numMilestones = int(math.ceil(tf*self.freq))
        # diff = [q0i-qfi for q0i,qfi in zip(q0,qf)]
        # print 'Diff = {}'.format([round(qi,2) for qi in diff])
        # print 'Q: {:.15f} s, {} --> {}'.format(tf,q0,qf)
        # print 'Calculated {} milestones'.format(numMilestones)
        for t in np.linspace(dt, tf, numMilestones):
            q = self.interpolate(q0, qf, t, tf)
            m = Milestone(t=dt,robot=q)
            # print "appending {}".format((dt,q))
            milestones.append(m)
        print 'Added {} milestones'.format(len(milestones))
        return milestones

if __name__ == "__main__":
    s = PensiveClient().default()
    p = LinearPlanner()
    T = s.get('robot/inspect_pose')
    q = p.getDesiredConfig(T)
    p.interpolate(q=q)
