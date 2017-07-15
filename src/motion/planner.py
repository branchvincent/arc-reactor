from pensive.client import PensiveClient
from master.world import build_world, xyz, rpy, numpy2klampt, klampt2numpy
from motion.milestone import Milestone

from klampt.model import ik
from klampt.math import so3, se3, vectorops as vops
from klampt.model.collide import WorldCollider

from time import time
from math import pi, atan2, acos, sqrt, ceil, radians, degrees
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


class MotionPlanner:
    """
    High level motion planner to plan motions to goal transforms and configurations
    """

    def __init__(self, store=None):
        self.store = store or PensiveClient().default()
        self.reset()
        # Update current config


    def reset(self):
        # Init planners
        self.world = build_world(self.store)
        self.cspace = CSpace(self.world)
        self.se3space = SE3Space(self.world)
        self.joint_planner = JointPlanner(self.cspace, store=self.store)
        self.task_planner = TaskPlanner(self.se3space, store=self.store)
        self.vmax_abs = self.cspace.robot.getVelocityLimits()
        self.amax_abs = self.cspace.robot.getAccelerationLimits()
        self.plan = MotionPlan(self.cspace, store=self.store)
        # Update current config
        q0 = self.store.get('robot/current_config')
        self.store.put('planner/tracking/current_config', q0)
        # Set state
        self.setState('idle')

    def setState(self, state):
        logger.debug('Entering state {}'.format(state))
        self.store.put('planner/current_state', state.lower())

    def addMilestone(self, milestone):
        self.plan.addMilestone(milestone)
        q = milestone.get_robot()
        self.store.put('planner/tracking/current_config', q)

    def addMilestones(self, milestones):
        self.plan.addMilestones(milestones)
        if len(milestones) != 0:
            q = milestones[-1].get_robot()
            self.store.put('planner/tracking/current_config', q)

    def setVelocityLimits(self, vmax):
        # Set only if does not exceed absolute limits
        for i, (v, v_ceil) in enumerate(zip(vmax, self.vmax_abs)):
            vmax[i] = v if v < v_ceil else v_ceil
        self.robot.setVelocityLimits(vmax)

    def setAccelerationLimits(self, amax):
        # Set only if does not exceed absolute limits
        for i, (a, a_ceil) in enumerate(zip(amax, self.amax_abs)):
            amax[i] = a if a < a_ceil else a_ceil
        self.robot.setAccelerationLimits(amax)

    def getCurrentConfig(self):
        if len(self.plan.milestones) == 0:
            return self.store.get('robot/current_config')
        else:
            return self.plan.milestones[-1].get_robot()

    def checkCurrentConfig(self, strict=True):
        q0 = self.store.get('/planner/tracking/current_config')
        feasible = self.cspace.feasible(q0)
        if not feasible:
            logger.error('Current configuration is infeasible')
            self.plan.put(feasible=False)
            if strict:
                exit()
        return feasible

    def toTransform(self, T_ee):
        # Check current config
        self.checkCurrentConfig()
        # Create plan
        milestones = self.planToTransform(T_ee)
        self.addMilestones(milestones)
        self.plan.put()

    def pick(self, T_item):
        if isinstance(T_item, np.ndarray):
            T_item = numpy2klampt(T_item)
        self.checkCurrentConfig()

        # Get settings
        searchAngle = self.store.get('planner/states/picking/search_angle_increment', 10)
        approachDistance = self.store.get('planner/states/picking/approach_distance', 0.05)
        delay = self.store.get('planner/states/picking/delay', 1.5)

        # Determine pick pose
        ee_local = self.store.get('planner/ee_local')
        T_pick = list(numpy2klampt(klampt2numpy(T_item) * rpy(pi, 0, 0)))  # flip z
        R_ee = self._getEndEffectorRotation(T_pick[1])
        R_ee_normal = self._getRotationMatchingAxis(R_ee, T_pick[0], axis='z')
        T_pick[0] = R_ee_normal
        T_pick[1] = se3.apply(T_pick, [-i for i in ee_local])

        # Move above item
        logger.debug('Moving over item')
        state = 'idle'
        self.setState(state)
        clearance_height = self.store.get(['planner', 'states', state, 'clearance_height'])
        T_above = (R_ee, [T_item[1][0], T_item[1][1], clearance_height])
        milestones = self.planToTransform(T_above, name='pick_above')
        self.addMilestones(milestones)

        # Move to item normal
        logger.debug('Moving to item normal')
        self.setState('picking_approach')
        T_pick_no_normal = (R_ee, T_pick[1])
        T_pick = self._getFeasiblePickTransform(T_pick, T_pick_no_normal, searchAngle)
        T_pick_approach = (T_pick[0], se3.apply(T_pick, [0, 0, -approachDistance]))
        milestones = self.planToTransform(T_pick_approach, name='pick_approach')
        self.addMilestones(milestones)

        # Lower ee
        logger.debug('Lowering end effector')
        self.setState('picking')
        milestones = self.planToTransform(T_pick, name='pick')
        self.addMilestones(milestones)

        # Pick up
        logger.debug('Picking')
        state = self.store.get('/planner/current_state')
        delay = self.store.get(['planner', 'states', state, 'delay'])
        vacuum = self.store.get(['planner', 'states', state, 'vacuum'])
        self.addMilestone(Milestone(t=delay, robot=self.getCurrentConfig(), vacuum=vacuum))

        # Raise ee back to item normal
        logger.debug('Raising end effector')
        t_departure = vops.add(T_pick[1], [0, 0, approachDistance])
        milestones = self.planToTransform((T_pick[0], t_departure))
        self.addMilestones(milestones)

        # Raise ee 
        logger.debug('Raising end effector')
        self.setState('picking_retraction')
        milestones = self.planToTransform(T_above)
        self.addMilestones(milestones)

    def pickToInspect(self, T_item):
        # Pick item
        self.pick(T_item)

        # Move to inspection station
        logger.debug('Moving to inspection station')
        self.setState('carrying')
        T_inspect = self.store.get('/robot/inspect_pose')
        milestones = self.planToTransform(T_inspect)
        self.addMilestones(milestones)
        self.plan.put()

    def stow(self, T_stow):
        if isinstance(T_stow, np.ndarray):
            T_stow = numpy2klampt(T_stow)
        self.checkCurrentConfig()

        # Move over stow location
        logger.debug('Moving over item')
        state = 'carrying'
        self.setState(state)
        clearance_height = self.store.get(['planner', 'states', state, 'clearance_height'])
        T_above = (T_stow[0], [T_stow[1][0], T_stow[1][1], clearance_height])
        milestones = self.planToTransform(T_above, name='stow_above')
        self.addMilestones(milestones)

        # Lower ee
        logger.debug('Lowering end effector')
        self.setState('stowing_approach')
        milestones = self.planToTransform(T_stow, name='stow')
        self.addMilestones(milestones)

        # Place
        logger.debug('Placing')
        state = 'stowing'
        self.setState(state)
        delay = self.store.get(['planner', 'states', state, 'delay']])
        vacuum = self.store.get(['planner', 'states', state, 'vacuum'])
        self.addMilestone(Milestone(t=delay, robot=self.getCurrentConfig(), vacuum=vacuum))

        # Raise ee
        logger.debug('Raising end effector')
        self.setState('stowing_retraction')
        milestones = self.planToTransform(T_above)
        self.addMilestones(milestones)
        self.plan.put()

    def planToTransform(self, T, via=[], name=None):
        if self.store.get('/planner/debug') and name:
            self.store.put(['vantage', name], klampt2numpy(T))

        # Get inputs
        state = self.store.get('/planner/current_state')
        q0 = self.store.get('/planner/tracking/current_config')
        space = self.store.get(['planner', 'states', state, 'planning_space'])
        solvers = self.store.get(['planner', 'states', state, 'planning_solvers'])

        # Choose planner
        if space == 'joint':
            planner = self.joint_planner
        elif space == 'task':
            planner = self.task_planner
        else:
            raise RuntimeError('Unrecognized space: {}'.format(space))

        # Plan
        plan = MotionPlan(self.cspace, store=self.store)
        for Ti in via + [T]:
            for i, solver in enumerate(solvers):
                self.store.put('/planner/tracking/current_solver', solver)
                if space == 'task' and solver != 'nearby':
                    logger.warn('Task space with {} solver requested. Intentional?'.format(solver))
                milestones = planner.planToTransform(Ti)
                # HACK: try again, assuming wrist flipped
                if milestones is None and space == 'task' and solver == 'nearby':
                    milestones = self._fixWristFlip(Ti, planner.T_failed)
                # Update milestones, if found
                if milestones is not None:
                    plan.addMilestones(milestones)
                    break
                # Return, if all solvers failed
                elif i == len(solvers) - 1:
                    logger.error('Infeasible Goal Transform: {}'.format(Ti))
                    self.store.put('vantage/infeasible', klampt2numpy(Ti))
                    return None
                else:
                    logger.warning(
                        'IK {} solver failed. Trying {} solver...'.format(solver, solvers[i + 1]))
        # Check feasibility
        # if plan.feasible():
        #     logger.debug('Created {} milestones'.format(len(plan.milestones)))
        #     return plan.milestones
        # else:
        #     return None
        return plan.milestones

    def planToConfig(self, q, via=[]):
        # Get inputs
        state = self.store.get('/planner/current_state')
        q0 = self.store.get('/planner/tracking/current_config')
        space = self.store.get(['planner', 'states', state, 'planning_space'])
        solvers = self.store.get(['planner', 'states', state, 'planning_solvers'])

        # Choose planner
        if space == 'joint':
            planner = self.joint_planner
        elif space == 'task':
            planner = self.task_planner
        else:
            raise RuntimeError('Unrecognized space: {}'.format(space))

        # Plan
        plan = MotionPlan(self.cspace, store=self.store)
        for qi in via + [q]:
            for i, solver in enumerate(solvers):
                self.store.put('/planner/tracking/current_solver', solver)
                milestones = planner.planToConfig(qi)
                # Update milestones, if found
                if milestones is not None:
                    plan.addMilestones(milestones)
                    break
                # Exit, if all solvers failed
                elif i == len(solvers) - 1:
                    logger.error(
                        'Infeasible Goal Configuration: {}'.format(qi))
                    return None
                else:
                    logger.warning(
                        'IK {} solver failed. Trying {} solver...'.format(solver, solvers[i + 1]))
        # Check feasibility
        # if plan.feasible():
        #     logger.debug('Created {} milestones'.format(len(plan.milestones)))
        #     return plan.milestones
        # else:
        #     return None
        return plan.milestones

    def _getEndEffectorRotation(self, t):
        Rz = atan2(t[1], t[0]) - pi
        return numpy2klampt(xyz(*t) * rpy(pi, 0, 0) * rpy(0, 0, -Rz))[0]

    def _getRotationMatchingAxis(self, R, Rmatch, axis='x'):
        i = 0 if axis == 'x' else 1 if axis == 'y' else 2
        axis = [0] * 3
        axis[i] = 1
        v = so3.apply(R, axis)
        vm = so3.apply(Rmatch, axis)
        Rf = so3.vector_rotation(v, vm)
        return so3.mul(Rf, R)

    def _getFeasiblePickTransform(self, T_pick_normal, T_pick_no_normal, searchAngle):
        d = so3.distance(T_pick_normal[0], T_pick_no_normal[0])
        numAttempts = 2 + ceil(d/radians(searchAngle))
        T_pick_attempts = [se3.interpolate(T_pick_normal, T_pick_no_normal, u) for u in np.linspace(0, 1, numAttempts)]
        for i, Ti in enumerate(T_pick_attempts):
            logger.debug('Trying normal interpolation {} of {}'.format(i + 1, numAttempts))
            milestones = self.planToTransform(Ti)
            if milestones is not None:
                return Ti

    def _fixWristFlip(self, T, T_failed):
        logger.warn('Detected possible wrist flip. Trying hack...')
        self.store.put('/planner/tracking/current_solver', 'nearby')

        # Plan until failed T
        plan = MotionPlan(self.cspace, store=self.store)
        Ti_mid = list(numpy2klampt(klampt2numpy(T_failed) * rpy(0, 0, pi)))  # flip z
        milestones = self.task_planner.planToTransform(Ti_mid)
        if milestones is None: return None
        plan.addMilestones(milestones)
        # logger.debug('Fixed up to T_mid')

        # Plan until final T
        milestones = self.task_planner.planToTransform(T)
        if milestones is None: return None
        plan.addMilestones(milestones)
        logger.debug('Hack succeeded')
        return plan.milestones


class LowLevelPlanner(object):
    """
    Low level planner that solves for configurations and creates motion plans
    """

    def __init__(self, world, store=None):
        self.world = world
        self.robot = self.world.robot('tx90l')
        self.store = store or PensiveClient().default()

    # def planToTransform(self):
    #     raise NotImplementedError
    # def planToConfig(self):
    #     raise NotImplementedError

    def solveForConfig(self, T):
        """Solves for desired configuration to reach the specified end effector transform"""
        if isinstance(T, np.ndarray):
            T = numpy2klampt(T)
        ee_link = self.robot.link(self.robot.numLinks() - 1)
        goal = ik.objective(ee_link, R=T[0], t=T[1])
        return self.solve(goal)

    def solve(self, goals):
        solver = self.store.get('/planner/tracking/current_solver')
        q0 = self.store.get('/planner/tracking/current_config')
        eps = self.store.get('/planner/nearby_solver_eps', pi/8)

        # Solve
        if solver == 'local':
            solved = ik.solve(goals)
        elif solver == 'global':
            solved = ik.solve_global(goals)
        elif solver == 'nearby':
            solved = ik.solve_nearby(
                goals, eps, feasibilityCheck=lambda: True)
        else:
            raise RuntimeError('Unrecognized solver: {}'.format(solver))

        # Return solution
        if solved:
            return self.robot.getConfig()
        else:
            # logger.warning(
            #     'IK {} solver failed'.format(solver))
            # NOTE: this change may cause error
            # self.robot.setConfig(q0)
            return None


class JointPlanner(LowLevelPlanner):
    """
    Planner that works in joint (configuration) space
    """

    def __init__(self, space, store=None):
        super(JointPlanner, self).__init__(space.world, store=store)
        self.reset(space)

    def reset(self, cspace):
        self.space = cspace

    def planToTransform(self, T):
        q0 = self.store.get('/planner/tracking/current_config')
        q = self.solveForConfig(T)
        if q is None:
            return None
        else:
            return self.planToConfig(q)

    def planToConfig(self, q):
        # Get inputs
        state = self.store.get('/planner/current_state')
        q0 = self.store.get('/planner/tracking/current_config')
        freq = self.store.get('/planner/control_frequency')
        profile = self.store.get('/planner/velocity_profile')
        vmax = self.store.get(['planner', 'states', state, 'joint_velocity_limits'])
        amax = self.store.get(['planner', 'states', state, 'joint_acceleration_limits'])
        vacuum = self.store.get(['planner', 'states', state, 'vacuum'])

        # Calculate duration
        q = self.space.getGeodesic(q0, q)
        tf = self.space.getMinPathTime(q0, q, vmax=vmax, amax=amax, profile=profile)
        dt = 1 / float(freq)
        if tf != 0:
            tf += abs(tf % dt - dt)     # make multiple of dt
            # tf = max(tf, 1)             # enforce min time

        # Add milestones
        milestones = []
        numMilestones = int(ceil(tf * freq))
        for t in np.linspace(dt, tf, numMilestones):
            qi = self.space.interpolate(q0, q, t, tf, vmax=vmax, amax=amax, profile=profile)
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

    def __init__(self, se3space, store=None):
        super(TaskPlanner, self).__init__(se3space.world, store=store)
        self.reset(se3space)

    def reset(self, se3space):
        self.space = se3space
        self.ee_link = self.robot.link(self.robot.numLinks() - 1)
        # self.vmax = 0.15
        # self.t_vmax, self.t_amax = 0.15, 0.15
        # self.R_vmax, self.t_amax = pi / 4, pi / 4
        self.T_failed = None

    def planToTransform(self, T):
        # Get inputs
        state = self.store.get('/planner/current_state')
        q0 = self.store.get('/planner/tracking/current_config')
        vmax = self.store.get(['planner', 'states', state, 'translation_velocity_limit'])
        freq = self.store.get('/planner/control_frequency')
        profile = self.store.get('/planner/velocity_profile')
        vacuum = self.store.get(['planner', 'states', state, 'vacuum'])

        if isinstance(T, np.ndarray):
            T = numpy2klampt(T)

        # Calculate duration
        self.robot.setConfig(q0)
        T0 = self.ee_link.getTransform()
        # tf = self.space.getMinPathTime(T0, T, profile=self.profile)
        # add extra second for ramp up/down
        tf = vops.distance(T0[1], T[1]) / vmax
        # logger.debug('Distance from {} to {} = {}'.format(T0[0], T[1], tf*self.vmax))
        dt = 1 / float(freq)
        if tf != 0:
            tf += abs(tf % dt - dt)     # make multiple of dt
            tf = max(tf, 1)             # enforce min time for ramp up/down

        # Add milestones
        milestones = []
        numMilestones = int(ceil(tf * freq))
        for t in np.linspace(dt, tf, numMilestones):
            Ti = self.space.interpolate(T0, T, t, tf, profile=profile)
            q = self.solveForConfig(Ti)
            if q is None:
                self.T_failed = Ti
                self.store.put('vantage/failed', klampt2numpy(self.T_failed))
                return None
            # Add milestone
            m = Milestone(t=dt, robot=q, vacuum=vacuum)
            milestones.append(m)
            # if not self.cspace.feasible(m.get_robot()):
            #     logger.warn('Not feasible')
        return milestones

    def planToConfig(self, q):
        # q = self.cspace.getGeodesic(q0, q)
        self.robot.setConfig(q)
        Tf = self.ee_link.getTransform()
        return self.planToTransform(Tf)


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
        # self.failed_milestones = []

    def feasible(self):
        for milestone in self.milestones:
            if not self.cspace.feasible(milestone.get_robot()):
                return False
        return True

    def getFeasibleMilestones(self):
        feasible_milestones = []
        for milestone in self.milestones:
            if self.cspace.feasible(milestone.get_robot()):
                feasible_milestones.append(milestone)
            else:
                break
        return feasible_milestones

    def getCurrentConfig(self):
        if len(self.milestones) != 0:
            return self.milestones[-1].get_robot()
        return None

    def addMilestone(self, milestone, strict=True):
        if milestone is not None:
            self.milestones.append(milestone)
        elif strict:
            self.put(feasible=False)
            exit()

    def addMilestones(self, milestones, strict=True):
        if milestones is not None:
            self.milestones += milestones
        elif strict:
            self.put(feasible=False)
            exit()

    def put(self, feasible=None):
        # Check feasibility
        if feasible is None:
            feasible = self.feasible()

        # Update database
        if feasible:
            logger.info('Feasible path found. Updating waypoints...')
            milestoneMap = [m.get_milestone() for m in self.milestones]
            self.store.put('/robot/waypoints', milestoneMap)
            self.store.put('/robot/timestamp', time())
        else:
            logger.error('Failed to find feasible path. Clearing waypoints...')
            # Clear waypoints
            self.store.put('/robot/waypoints', None)
            self.store.put('/robot/timestamp', time())
            # Update debug waypoints
            milestoneMap = [m.get_milestone()
                            for m in self.getFeasibleMilestones()]
            self.store.put('/debug/waypoints', milestoneMap)
            self.store.put('/debug/timestamp', time())


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

    def interpolate(self, q0, qf, t, tf, vmax=None, amax=None, profile='linear'):
        """Interpolates between q0 and qf, with the specified profile"""
        profile = profile.lower()
        r = t / tf
        if profile == 'linear':
            u = r
            return self.robot.interpolate(q0, qf, u)
        elif profile == 'cubic':
            u = 3 * r**2 - 2 * r**3
            return self.robot.interpolate(q0, qf, u)
        elif profile == 'quintic':
            u = 10 * r**3 - 15 * r**4 + 6 * r**5
            return self.robot.interpolate(q0, qf, u)
        elif profile == 'bang-bang':
            u = 2 * r**2 if 0 <= r <= 0.5 else -1 + 4 * r - 2 * r**2
            return self.robot.interpolate(q0, qf, u)
        elif profile == 'trapeze':
            vmax = vmax or self.robot.getVelocityLimits()
            amax = amax or self.robot.getAccelerationLimits()
            q = q0[:]
            for i, (q0i, qfi, vi, ai) in enumerate(zip(q0, qf, vmax, amax)):
                tau = vi / ai if ai != 0 else 0
                Di = qfi - q0i
                Dsign = np.sign(Di)
                if 0 <= t < tau:
                    q[i] = q0i + 0.5 * t**2 * ai * Dsign
                elif tau <= t < tf - tau:
                    q[i] = q0i + (t - tau / 2) * vi * Dsign
                else:
                    q[i] = qfi - 0.5 * (tf - t)**2 * ai * Dsign
            return q
        else:
            raise RuntimeError(
                'Unrecognized interpolation profile: {}'.format(profile))

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
            elif profile == 'trapeze':
                tf.append(vi / ai + D / vi if vi and ai != 0 else 0)
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

    def getMinPathTime(self, q0, qf, vmax=None, amax=None, profile='linear'):
        """
        Returns the minimum path time between q0 and qf, while respecting joint
        velocity/accleration constraints, and the specified profile
        """
        # TODO
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


if __name__ == "__main__":
    s = PensiveClient().default()
    p = MotionPlanner()
