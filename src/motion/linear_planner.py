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

    def getDesiredConfig(self, T=None, p=None, global_solve=False):
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

        if global_solve:
            result = ik.solve_global(goal)
        else:
            result = ik.solve(goal)

        if result:
            return self.robot.getConfig()
        else:
            logger.warn('Could not find feasible configuration')
            raise RuntimeError('Could not find feasible configuration')
            # return self.robot.getConfig()

    def interpolate(self, q=None, T=None, p=None, global_solve=False, put=True):
        # if (q is None and T is None and p is None):
        #     raise RuntimeError('Cannot interpolate nothing')

        """Jogs the robot to the specified configuration, in radians"""
        # Get initial configuration
        q0 = self.store.get('/robot/current_config')
        # print "T is ", T
        print 'Joint limits', self.robot.getJointLimits()

        if T is not None:
            q = self.getDesiredConfig(T=T, global_solve=global_solve)
            # print "T is not none so q is ", q
        elif p is not None:
            q = self.getDesiredConfig(p=p, global_solve=global_solve)
        elif q is None:
            raise RuntimeError('Must specify either p, q, or T')

        print "q is ", q
        # Get milestones and check feasibility
        feasible = True
        milestones = TimeScale(self.robot).getMilestones(q0, q)
        for m in milestones:
            feasible = self.isFeasible(m.get_robot())
            if not feasible:
                break

        milestoneMap = [m.get_milestone() for m in milestones]
        if put:
            self.store.put('/robot/waypoints', milestoneMap)
            self.store.put('/robot/timestamp', time.time())

        # Update database
        if feasible:
            self.store.put('/status/route_plan', True)
        else:
            self.store.put('/status/route_plan', False)
            logger.warn('Could not find feasible path')

        return milestones

class TimeScale:
    def __init__(self, robot, type='cubic', freq=20):
        self.robot = robot
        self.vmax = robot.getVelocityLimits()
        self.amax = robot.getAccelerationLimits()
        #HACK: incorrect a,v limits?
        # k = 8
        # self.vmax = [k*math.radians(vi) for vi in self.vmax]
        # self.amax = [k*math.radians(ai) for ai in self.amax]
        self.vmax = [math.radians(60)]*len(self.vmax)
        self.amax = [math.radians(60)]*len(self.amax)
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
        return vops.add(q0, vops.mul(D, r))

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
    from hardware.control.robotcontroller import RobotController
    from hardware.control.simulatedrobotcontroller import SimulatedRobotController

    def executePlan():
        # Query
        question = lambda: str(raw_input("Execute path? (y/n): ")).lower().strip()[0]
        execute = question()
        while execute not in ['y','n']:
            execute = question()

        # Execute
        if execute == 'y':
            print "Executing..."
            SimulatedRobotController().run()
        else:
            print "Not executing..."

    s = PensiveClient().default()
    p = LinearPlanner()

    T = s.get('robot/inspect_pose')
    # q = p.getDesiredConfig(T)
    p.interpolate(T=T)

    # T = s.get('shelf/bin/binC/vantage')
    # # p.interpolate(T=T)
    # # executePlan()

    # # Get image
    # from hardware.SR300 import DepthCameras
    # from states.find_item import FindItem
    # import cv2
    # import numpy
    # state = FindItem('fi')

    # # connect cameras
    # cameras = DepthCameras()
    # if not cameras.connect():
    #     raise RuntimeError('failed accessing cameras')
    # camera_serials = state.store.get('system/cameras')
    # camera = 'tcp'

    # c,ac,pc = state.acquire_image(cameras, camera, camera_serials)
    # cv2.imwrite('data/simulation/color-{}-0.png'.format(camera), c[:, :, ::-1])
    # cv2.imwrite('data/simulation/aligned-{}-0.png'.format(camera), ac[:, :, ::-1])
    # numpy.save('data/simulation/pc-{}-0.npy'.format(camera), pc)
