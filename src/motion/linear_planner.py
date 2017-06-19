from pensive.client import PensiveClient
from master.world import build_world, xyz, rpy, numpy2klampt, klampt2numpy
# from util.sync_robot import sync
from motion.milestone import Milestone

from klampt.model import ik
from klampt.math import se3, vectorops as vops
from klampt.model.collide import WorldCollider
# from klampt.plan.robotcspace import RobotCSpace

from time import time
from math import pi, atan2, sqrt, ceil, radians
import numpy as np
import logging

logger = logging.getLogger(__name__)

# TODO: for se3, scale t if out of a/v limits

class InfeasibleGoalError(Exception):
    pass
class CollisionError(Exception):
    pass
class JointLimitError(Exception):
    pass


class Planner:
    """
    High level planner to plan motions to goal transforms and configurations
    """

    def __init__(self, store=None):
        self.store = store or PensiveClient().default()
        self.ee_local = [0, 0, 0.4]
        self.movement_plane = 1
        self.reset()

    def reset(self):
        self.world = build_world(self.store)
        self.cspace = CSpace(self.world)
        self.se3space = SE3Space(self.world)
        self.joint_planner = JointPlanner(self.cspace, store=self.store, profile='cubic', freq=20)
        self.task_planner = TaskPlanner(self.se3space, store=self.store, profile='cubic', freq=20)
        self.plan = MotionPlan(self.cspace, store=self.store)

    def setVacuum(self, value):
        self.joint_planner.setVacuum(value)
        self.task_planner.setVacuum(value)

    def put(self):
        self.plan.put()

    def putFeasible(self):
        self.plan.putFeasible()

    def getTAbove(self, x):
        x_over = x[0:2] + [self.movement_plane]
        Rz = atan2(x_over[1], x_over[0]) - pi
        return numpy2klampt(xyz(*x_over) * rpy(pi,0,0) * rpy(0,0,-Rz))

    def pick_up(self, T_item, pick_time=0.5):
        if isinstance(T_item, np.ndarray):
            T_item = numpy2klampt(T_item)
        logger.info('T_item {}'.format(T_item))
        # self.store.put('vantage/item', klampt2numpy(T_item))

        # Move over item
        logger.info('Moving over item')
        x_item = T_item[1]
        T_over_item = self.getTAbove(x_item)
        # self.store.put('vantage/overhead', klampt2numpy(T_over_item))
        milestones = self.planToTransform(T_over_item, space='joint', solvers=['local', 'global'])

        # Lower ee
        logger.info('Lowering end effector')
        q0 = milestones[-1].get_robot()
        T_item = T_over_item[0], vops.add(x_item, self.ee_local)
        milestones = self.planToTransform(T_item, q0=q0, vacuum=[1], space='task', solvers=['nearby', 'local'])

        # Pick up
        q0 = milestones[-1].get_robot()
        self.plan.addMilestone(Milestone(t=pick_time, robot=q0, vacuum=[0]))

        # Raise ee
        logger.info('Raising end effector')
        q0 = milestones[-1].get_robot()
        milestones = self.planToTransform(T_over_item, q0=q0, vacuum=[1], space='task', solvers=['nearby', 'local'])

        # Move to inspection station
        logger.info('Moving to inspection station')
        T_inspect = self.store.get('/robot/inspect_pose')
        q0 = milestones[-1].get_robot()
        milestones = self.planToTransform(T_inspect, q0=q0, vacuum=[1], space='task', solvers=['nearby', 'local'])
        self.put()
        return self.plan.milestones

    def drop_item(self, T_drop, drop_time=1):
        if isinstance(T_drop, np.ndarray):
            T_drop = numpy2klampt(T_drop)

        # Move over item
        x_drop = T_drop[1]
        T_over_drop = self.getTAbove(x_drop)
        self.store.put('vantage/overhead', klampt2numpy(T_over_drop))
        milestones = self.planToTransform(T_over_drop, vacuum=[1], space='joint', solvers=['local', 'global'])

        # Move to drop position
        q0 = milestones[-1].get_robot()
        T_item = T_over_drop[0], vops.add(x_drop, self.ee_local)
        milestones = self.planToTransform(T_drop, q0=q0, vacuum=[1], space='task', solvers=['nearby', 'local'])

        # Release
        q0 = milestones[-1].get_robot()
        self.plan.addMilestone(Milestone(t=drop_time, robot=q0, vacuum=[0]))

        # Raise
        q0 = milestones[-1].get_robot()
        milestones = self.planToTransform(T_over_drop, q0=q0, space='task', solvers=['local', 'global'])
        self.put()
        return self.plan.milestones

    def stow_grab(self, item):
        return self.pick_up(item)

    def place_shelf(self, T_placement):
        return self.drop_item(T_placement)

    def planToTransform(self, T, via=[], q0=None, vacuum=[0], space='joint', solvers=['local']):
        # Choose planner
        space = space.lower()
        if space == 'joint':
            planner = self.joint_planner
        elif space == 'task':
            planner = self.task_planner
        else:
            raise RuntimeError('Unrecognized space: {}'.format(space))

        # Plan
        for Ti in via + [T]:
            for i, solver in enumerate(solvers):
                milestones = planner.planToTransform(Ti, q0=q0, vacuum=vacuum, solver=solver)
                # Update milestones, if found
                if milestones:
                    self.plan.addMilestones(milestones)
                    q0 = milestones[-1].get_robot()
                    break
                # Return, if all solvers failed
                elif i == len(solvers) - 1:
                    logger.error('Infeasible Goal Transform: {}'.format(Ti))
                    return None
                else:
                    logger.warning(
                        'IK {} solver could not find feasible configuration. Trying {} solver...'.format(solver, solvers[i+1]))
        logger.info('Created {} milestones'.format(len(self.plan.milestones)))
        return self.plan.milestones

    def planToConfiq(self, q, via=[], q0=None, vacuum=[0], space='joint', solvers=['local']):
        # Choose planner
        space = space.lower()
        if space == 'joint':
            planner = self.joint_planner
        elif space == 'task':
            planner = self.task_planner
        else:
            raise RuntimeError('Unrecognized space: {}'.format(space))

        # Plan
        for qi in via + [q]:
            for i, solver in enumerate(solvers):
                milestones = planner.planToConfig(qi, q0=q0, vacuum=vacuum, solver=solver)
                # Update milestones, if found
                if milestones:
                    self.plan.addMilestones(milestones)
                    q0 = milestones[-1].get_robot()
                    break
                # Exit, if all solvers failed
                elif i == len(solvers) - 1:
                    logger.error(
                        'Infeasible Goal Configuration: {}'.format(qi))
                    return None
                else:
                    logger.warning(
                        'IK {} solver could not find feasible configuration. Trying {} solver...'.format(solver, solvers[i+1]))
        logger.info('Created {} milestones'.format(len(self.plan.milestones)))
        return self.plan.milestones


class LowLevelPlanner(object):
    """
    Low level planner that solves for configurations and creates motion plans
    """

    def __init__(self, world, store=None, profile='cubic', freq=20):
        self.world = world
        self.robot = self.world.robot('tx90l')
        self.store = store or PensiveClient().default()
        self.profile = profile.lower()
        self.freq = freq
        self.vacuum = 0

    def setVacuum(self, value):
        self.vacuum = int(value)

    def setProfile(self, profile):
        self.profile = profile.lower()

    def setFrequency(self, freq):
        self.freq = freq

    # def planToTransform(self):
    #     raise NotImplementedError
    #
    # def planToConfig(self):
    #     raise NotImplementedError

    def solveForConfig(self, T, solver='local', eps=None):
        """Solves for desired configuration to reach the specified end effector transform"""
        if isinstance(T, np.ndarray):
            T = numpy2klampt(T)
        ee_link = self.robot.link(self.robot.numLinks() - 1)
        goal = ik.objective(ee_link, R=T[0], t=T[1])
        return self.solve(goal, solver=solver, eps=eps)

    def solve(self, goals, solver='local', eps=None):
        solver = solver.lower()
        q0 = self.robot.getConfig()

        # Solve
        if solver == 'local':
            result = ik.solve(goals)
        elif solver == 'global':
            result = ik.solve_global(goals)
        elif solver == 'nearby':
            result = ik.solve_nearby(goals, eps or pi/8, feasibilityCheck=lambda: True)
        else:
            raise RuntimeError('Unrecognized solver: {}'.format(solver))

        # Return solution
        if result:
            return self.robot.getConfig()
        else:
            # logger.warning(
            #     'IK {} solver could not find feasible configuration'.format(solver))
            self.robot.setConfig(q0)
            return None


class JointPlanner(LowLevelPlanner):
    """
    Planner that works in joint (configuration) space
    """

    def __init__(self, space, store=None, profile='cubic', freq=20):
        super(JointPlanner, self).__init__(
            space.world, store=store, profile=profile, freq=freq)
        self.reset(space)

    def reset(self, space):
        self.space = space
        # Update v, a limits
        avmax = [pi / 3] * self.robot.numLinks()
        self.space.setAccelerationLimits(avmax)
        self.space.setVelocityLimits(avmax)

    def planToTransform(self, T, q0=None, vacuum=[0], solver='local', eps=None):
        q0 = q0 or self.store.get('/robot/current_config')
        q = self.solveForConfig(T, solver=solver, eps=eps)
        if q is None:
            return None
        else:
            return self.planToConfig(q, q0=q0, vacuum=vacuum, solver=solver)

    def planToConfig(self, q, q0=None, vacuum=[0], solver='local'):
        q0 = q0 or self.store.get('/robot/current_config')
        q = self.space.getGeodesic(q0, q)

        # Calculate duration
        tf = self.space.getMinPathTime(q0, q, profile=self.profile)
        dt = 1 / float(self.freq)
        if tf != 0:
            tf += abs(tf % dt - dt)     # make multiple of dt
            # tf = max(tf, 1)             # enforce min time

        # Add milestones
        milestones = []
        numMilestones = int(ceil(tf * self.freq))
        for t in np.linspace(dt, tf, numMilestones):
            qi = self.space.interpolate(q0, q, t, tf, profile=self.profile)
            m = Milestone(t=dt, robot=qi, vacuum=vacuum)
            milestones.append(m)
            # if not self.space.feasible(m.get_robot()):
            #     logger.warn('Not feasible')
        # logger.debug('Created {} milestones'.format(len(milestones)))
        return milestones


class TaskPlanner(LowLevelPlanner):
    """
    Planner that works in task (se3) space
    """

    def __init__(self, se3space, store=None, profile='cubic', freq=20):
        super(TaskPlanner, self).__init__(
            se3space.world, store=store, profile=profile, freq=freq)
        self.reset(se3space)

    def reset(self, se3space):
        self.space = se3space
        self.ee_link = self.robot.link(self.robot.numLinks() - 1)
        self.vmax = 0.25

    def planToTransform(self, T, q0=None, vacuum=[0], solver='local', eps=None):
        if isinstance(T, np.ndarray):
            T = numpy2klampt(T)
        q0 = q0 or self.store.get('/robot/current_config')

        # Calculate duration
        self.robot.setConfig(q0)
        T0 = self.ee_link.getTransform()
        # tf = self.space.getMinPathTime(T0, T, profile=self.profile)
        tf = vops.distance(T0[1], T[1]) / self.vmax
        # logger.debug('Distance from {} to {} = {}'.format(T0[0], T[1], tf*self.vmax))
        dt = 1 / float(self.freq)
        if tf != 0:
            tf += abs(tf % dt - dt)     # make multiple of dt
            # tf = max(tf, 1)             # enforce min time

        # Add milestones
        milestones = []
        numMilestones = int(ceil(tf * self.freq))
        for t in np.linspace(dt, tf, numMilestones):
            Ti = self.space.interpolate(T0, T, t, tf, profile=self.profile)
            q = self.solveForConfig(Ti, solver=solver, eps=eps)
            m = Milestone(t=dt, robot=q, vacuum=vacuum)
            milestones.append(m)
            # if not self.cspace.feasible(m.get_robot()):
            #     logger.warn('Not feasible')
        return milestones

    def planToConfig(self, q, q0=None, vacuum=[0], solver='local', eps=None):
        q0 = q0 or self.store.get('/robot/current_config')
        # q = self.cspace.getGeodesic(q0, q)
        self.robot.setConfig(q)
        Tf = self.ee_link.getTransform()
        return self.planToTransform(Tf, q0=q0, vacuum=vacuum, solver=solver, eps=eps)


class MotionPlan:
    """
    A motion plan defined by a list of milestones
    """

    def __init__(self, cspace, store=None):
        self.store = store or PensiveClient().default()
        self.cspace = cspace
        self.reset()

    def reset(self):
        self.milestones = []

    def feasible(self):
        for milestone in self.milestones:
            if not self.cspace.feasible(milestone.get_robot()):
                return False
        return True

    def addMilestone(self, milestone):
        self.milestones.append(milestone)

    def addMilestones(self, milestones):
        self.milestones += milestones

    def putFeasible(self):
        milestoneMap = []
        for m in self.milestones:
            if self.cspace.feasible(m.get_robot()):
                milestoneMap.append(m.get_milestone())
            else:
                break
        self.store.put('/robot/waypoints', milestoneMap)

    def put(self):
        # Convert milestone to dictionary
        success = self.feasible()
        if success:
            milestoneMap = [m.get_milestone() for m in self.milestones]
        else:
            logger.error('Failed to find feasible path')
            milestoneMap = []
        # Update database
        self.store.put('/robot/waypoints', milestoneMap)
        self.store.put('/robot/timestamp', time())
        self.store.put('/status/route_plan', success)


class CSpace:
    """
    A configuration space used to determine feasible configurations
    """

    def __init__(self, world):
        self.world = world
        self.robot = world.robot('tx90l')
        self.collider = WorldCollider(world)
        self.feasibilityTests = [self.inJointLimits,
                                 self.noSelfCollision,
                                 self.noEnvCollision]

    def setVelocityLimits(self, vmax):
        self.robot.setVelocityLimits(vmax)

    def setAccelerationLimits(self, amax):
        self.robot.setAccelerationLimits(amax)

    def getGeodesic(self, q0, qf):
        """Returns a new goal configuration that follows along a geodesic"""
        qnew = qf[:]
        ql, qu = self.robot.getJointLimits()
        for i, (q0i, qfi, qli, qui) in enumerate(zip(q0, qf, ql, qu)):
            # Get candidate
            D = qfi - q0i
            qni = qfi - np.sign(D) * 2 * pi
            # Update, if applicable
            if not (-pi <= D <= pi) and (qli <= qni <= qui):
                logger.debug(
                    "Found Geodesic: delta {} changed to {}".format(D, qni - q0i))
                qnew[i] = qni
        return qnew

    def interpolate(self, q0, qf, t, tf, profile='linear'):
        """Interpolates between q0 and qf, with the specified profile"""
        profile = profile.lower()
        r = t / tf
        if profile == 'linear':
            u = r
            return self._interpolate(q0, qf, u)
        elif profile == 'cubic':
            u = 3 * r**2 - 2 * r**3
            return self._interpolate(q0, qf, u)
        elif profile == 'quintic':
            u = 10 * r**3 - 15 * r**4 + 6 * r**5
            return self._interpolate(q0, qf, u)
        elif profile == 'bang-bang':
            u = 2 * r**2 if 0 <= r <= 0.5 else -1 + 4 * r - 2 * r**2
            return self._interpolate(q0, qf, u)
        # elif profile == 'trapeze':
        #     D = vops.sub(qf, q0)
        #     vmax = vmax or self.robot.getVelocityLimits()
        #     amax = amax or self.robot.getAccelerationLimits()
        #     R = vops.div()
        #     for q0i, qfi, vi, ai in zip(q0, qf, vmax, amax):
        #         tau = vi/ai if ai != 0 else 0
        #         Di = qfi - q0i
        #         if 0 <= t < tau:
        #             qi = q0i + 0.5*t**2*ai*np.sign(Di)
        #         elif tau <= t < tf - tau:
        #             qi = q0i + (t - tau/2)*vi * np.sign(Di)
        #         else:
        #             qi = qfi - 0.5*(tf - t)**2 * ai * np.sign(Di)
        else:
            raise RuntimeError(
                'Unrecognized interpolation profile: {}'.format(profile))
        # return vops.add(q0, vops.mul(D, r))

    def _interpolate(self, q0, qf, u):
        D = vops.sub(qf, q0)
        # logger.info('Is {} == {}'.format(self.robot.interpolate(
        #     q0, qf, u), vops.add(q0, vops.mul(D, u))))
        assert sum(vops.sub(self.robot.interpolate(
            q0, qf, u), vops.add(q0, vops.mul(D, u)))) <= 0.1
        return vops.add(q0, vops.mul(D, u))

    def getMinPathTime(self, q0, qf, vmax=None, amax=None, profile='linear'):
        """
        Returns the minimum path time between q0 and qf, while respecting max
        velocity vmax, max acceleration amax, and the specified profile
        """
        profile = profile.lower()
        vmax = vmax or self.robot.getVelocityLimits()
        amax = amax or self.robot.getAccelerationLimits()
        tf = []

        for q0i, qfi, vi, ai in zip(q0, qf, vmax, amax):
            D = abs(qfi - q0i)
            if profile == 'linear':
                tf.append(D / vi if vi != 0 else 0)
            elif profile == 'cubic':
                tf.append(max(1.5 * D / vi, sqrt(6 * D / ai))
                          if vi and ai != 0 else 0)
            elif profile == 'quintic':
                tf.append(max(1.875 * D / vi, sqrt(10 * D / (sqrt(3) * ai)))
                          if vi and ai != 0 else 0)
            elif profile == 'bang-bang':
                tf.append(max(2 * D / vi, 2 * sqrt(D / ai))
                          if vi and ai != 0 else 0)
            # elif profile == 'trapeze':
            #     tf.append(vi/ai + D/vi if vi and ai != 0 else 0)
            else:
                raise RuntimeError(
                    'Unrecognized interpolation profile: {}'.format(profile))
        return max(tf)

    def feasible(self, q):
        """Checks feasibility of q"""
        for test in self.feasibilityTests:
            if not test(q):
                return False
        return True

    def inJointLimits(self, q):
        """Checks that q respects joint limits"""
        ql, qu = self.robot.getJointLimits()
        for i, (qi, qli, qui) in enumerate(zip(q, ql, qu)):
            if not (qli <= qi <= qui):
                logger.error(
                    'Joint Limit Error: joint {} not in [{}, {}]'.format(i, qli, qui)),
                return False
        return True

    def noSelfCollision(self, q):
        """Checks for self collisions at q"""
        self.robot.setConfig(q)
        if self.robot.selfCollides():
            logger.error('Collision Error: robot colliding with self')
            return False
        return True

    def noEnvCollision(self, q):
        """Checks for collisions with environment at q"""
        r = self.robot.index
        # Check object collisions
        for o in xrange(self.world.numRigidObjects()):
            if any(self.collider.robotObjectCollisions(r, o)):
                name = self.world.rigidObject(o).getName()
                logger.error(
                    'Collision Error: robot colliding with {}'.format(name))
                return False
        # Check terrain collisions
        for o in xrange(self.world.numTerrains()):
            if any(self.collider.robotTerrainCollisions(r, o)):
                name = self.world.terrain(o).getName()
                logger.error(
                    'Collision Error: robot colliding with {}'.format(name))
                return False
        return True


class SE3Space:
    """
    An SE3 space used to plan cartesian movements
    """

    def __init__(self, world):
        self.world = world
        self.robot = world.robot('tx90l')
        self.collider = WorldCollider(world)

    def interpolate(self, T0, Tf, t, tf, profile='linear'):
        """Interpolates between T0 and Tf, with the specified profile"""
        profile = profile.lower()
        r = t / tf
        if profile == 'linear':
            u = r
            return se3.interpolate(T0, Tf, u)
        elif profile == 'cubic':
            u = 3 * r**2 - 2 * r**3
            return se3.interpolate(T0, Tf, u)
        elif profile == 'quintic':
            u = 10 * r**3 - 15 * r**4 + 6 * r**5
            return se3.interpolate(T0, Tf, u)
        elif profile == 'bang-bang':
            u = 2 * r**2 if 0 <= r <= 0.5 else -1 + 4 * r - 2 * r**2
            return se3.interpolate(T0, Tf, u)
        # elif profile == 'trapeze':
        #     D = vops.sub(qf, q0)
        #     vmax = vmax or self.robot.getVelocityLimits()
        #     amax = amax or self.robot.getAccelerationLimits()
        #     R = vops.div()
        #     for q0i, qfi, vi, ai in zip(q0, qf, vmax, amax):
        #         tau = vi/ai if ai != 0 else 0
        #         Di = qfi - q0i
        #         if 0 <= t < tau:
        #             qi = q0i + 0.5*t**2*ai*np.sign(Di)
        #         elif tau <= t < tf - tau:
        #             qi = q0i + (t - tau/2)*vi * np.sign(Di)
        #         else:
        #             qi = qfi - 0.5*(tf - t)**2 * ai * np.sign(Di)
        else:
            raise RuntimeError(
                'Unrecognized interpolation profile: {}'.format(profile))
        # return vops.add(q0, vops.mul(D, r))

    def getMinPathTime(self, q0, qf, vmax=None, amax=None, profile='linear'):
        """
        Returns the minimum path time between q0 and qf, while respecting joint
        velocity/accleration constraints, and the specified profile
        """
        #TODO
        pass
        # profile = profile.lower()
        # vmax = vmax or self.robot.getVelocityLimits()
        # amax = amax or self.robot.getAccelerationLimits()
        # tf = []
        #
        # for q0i, qfi, vi, ai in zip(q0, qf, vmax, amax):
        #     D = abs(qfi - q0i)
        #     if profile == 'linear':
        #         tf.append(D / vi if vi != 0 else 0)
        #     elif profile == 'cubic':
        #         tf.append(max(1.5 * D / vi, sqrt(6 * D / ai))
        #                   if vi and ai != 0 else 0)
        #     elif profile == 'quintic':
        #         tf.append(max(1.875 * D / vi, sqrt(10 * D / (sqrt(3) * ai)))
        #                   if vi and ai != 0 else 0)
        #     elif profile == 'bang-bang':
        #         tf.append(max(2 * D / vi, 2 * sqrt(D / ai))
        #                   if vi and ai != 0 else 0)
        #     # elif profile == 'trapeze':
        #     #     tf.append(vi/ai + D/vi if vi and ai != 0 else 0)
        #     else:
        #         raise RuntimeError(
        #             'Unrecognized interpolation profile: {}'.format(profile))
        # return max(tf)


def checkGeodesic():
    p = HighLevelPlanner()
    q0 = [0, 0, -0.5, 1, 0, 1, 0]
    qf = q0[:]; qf[-1] = 3 * pi / 2
    p.planToConfiq(qf, q0=q0)

if __name__ == "__main__":
    # checkGeodesic()
    s = PensiveClient().default()
    p = Planner()
    T_binA = s.get('vantage/binA')
    T_binB = s.get('vantage/binB')
    T_binC = s.get('vantage/binC')
    p.planToTransform(T_binC, solvers=['local', 'nearby', 'global'])
    p.putFeasible()
    from klampt.plan.robotplanning import planToCartesianObjective, planToConfig
    # plan = planToCartesianObjective(p.robot, )
    # q = [0]*7
    # plan = planToConfig(p.world, p.world.robot('tx90l'), q)
