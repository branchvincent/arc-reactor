from pensive.client import PensiveClient
from master.world import build_world, update_world, xyz, rpy, numpy2klampt, klampt2numpy
from motion.milestone import Milestone

from klampt.model import ik
from klampt.math import so3, se3, vectorops as vops
from klampt.model.collide import WorldCollider

from time import time
from math import pi, atan2, acos, sqrt, ceil, radians, degrees
import numpy as np
import logging

logger = logging.getLogger(__name__)

EE_BASE_INDEX = 6
SWIVEL_INDEX = 7

class PlannerFailure(Exception):
    pass


class MotionPlanner:
    """
    High level motion planner to plan motions to goal transforms and configurations
    """

    def __init__(self, store=None, world=None):
        self.store = store or PensiveClient().default()
        self.world = world
        self.reset()

    def reset(self):
        self.options = self.store.get('/planner')
        # Init planners
        self.world = update_world(self.store, self.world)
        self.robot = self.world.robot('tx90l')
        self.cspace = CSpace(self.world, store=self.store)
        self.se3space = SE3Space(self.world, store=self.store)
        self.joint_planner = JointPlanner(self.cspace, store=self.store, options=self.options)
        self.task_planner = TaskPlanner(self.se3space, store=self.store, options=self.options)
        self.plan = MotionPlan(self.cspace, store=self.store)
        # Update current config
        self.options['current_config'] = self.store.get('robot/current_config')
        self.options['swivel'] = None
        self.options['failed_pose'] = None

    def setState(self, state):
        logger.debug('Entering state {}'.format(state))
        self.options['current_state'] = state.lower()

    def addMilestone(self, milestone):
        self.plan.addMilestone(milestone, strict=True)
        q = milestone.get_robot_gripper()
        self.options['current_config'] = q
        self.robot.setConfig(q)

    def addMilestones(self, milestones):
        self.plan.addMilestones(milestones, strict=True)
        if len(milestones) != 0:
            q = milestones[-1].get_robot_gripper()
            self.options['current_config'] = q
            self.robot.setConfig(q)

    def getCurrentConfig(self):
        return self.options['current_config']

    def checkCurrentConfig(self):
        q0 = self.options['current_config']
        if not self.cspace.feasible(q0):
            logger.error('Current configuration is infeasible')
            self.plan.put(feasible=False)
            raise PlannerFailure('Current configuration is infeasible')

    def advanceAlongAxis(self, dist, axis='z'):
        # Find transform
        T_curr = list(numpy2klampt(self.store.get('/robot/tcp_pose')))
        # self.options['current_state'] = 'picking'
        i = 0 if axis == 'x' else 1 if axis == 'y' else 2
        vector = [0] * 3
        vector[i] = dist
        T_ee = [T_curr[0], se3.apply(T_curr, vector)]
        # Plan
        self.toTransform(T_ee)

    def toTransform(self, T_ee):
        self.checkCurrentConfig()
        milestones = self.planToTransform(T_ee)
        self.addMilestones(milestones)
        self.plan.put(feasible=True)

    def pick(self, T_item):
        # self.store.put('vantage/item', T_item)
        if isinstance(T_item, np.ndarray):
            T_item = numpy2klampt(T_item)
        self.checkCurrentConfig()

        # Get settings
        approachDistance = self.options['states']['picking']['approach_distance']
        retractDistance = self.options['states']['picking']['retract_distance']
        delay = self.options['states']['picking']['delay']

        ## Determine checkpoint poses
        # Pick pose
        q_pick, T_pick, T_pick_swivel = self._solveForPickConfig(T_item)
        swivel = q_pick[SWIVEL_INDEX]
        # Approach pose
        approach_offset_local = vops.sub(se3.apply(T_pick_swivel, [0, 0, -approachDistance]), T_pick_swivel[1])
        approach_offset = vops.add(T_pick[1], approach_offset_local)
        T_pick_approach = (T_pick[0], approach_offset)
        # Retract pose
        restract_offset = vops.add(T_pick[1], [0, 0, retractDistance])
        T_pick_departure = (T_pick[0], restract_offset)

        # Move above item
        logger.debug('Moving over item')
        self.setState('idle')
        self.options['swivel'] = 0
        T_initial = [T_pick[0], self._getInitialPickPose(T_item)[1]]
        # T_initial = self._getInitialPickPose(T_item)
        milestones = self.planToTransform(T_initial, link_index=EE_BASE_INDEX, name='pick_initial')
        self.addMilestones(milestones)

        # Move to item normal
        logger.debug('Descending to item normal')
        self.setState('picking_approach')
        self.options['swivel'] = swivel
        milestones = self.planToTransform(T_pick_approach, link_index=EE_BASE_INDEX, name='pick_approach')
        self.addMilestones(milestones)

        # Lower ee
        logger.debug('Descending along normal')
        self.setState('picking')
        milestones = self.planToTransform(T_pick, link_index=EE_BASE_INDEX, name='pick')
        self.addMilestones(milestones)
        self.store.put('/robot/target_grasp_xform', T_pick)

        # Pick up
        logger.debug('Picking')
        state = self.options['current_state']
        delay = self.options['states'][state]['delay']
        vacuum = self.options['states'][state]['vacuum']
        self.addMilestone(Milestone(t=delay, robot_gripper=self.getCurrentConfig(), vacuum=vacuum))

        # Raise ee back to item normal
        logger.debug('Ascending along normal')
        milestones = self.planToTransform(T_pick_departure, link_index=EE_BASE_INDEX)
        self.addMilestones(milestones)

        # Raise ee
        logger.debug('Ascending back over item')
        self.setState('picking_retraction')
        self.options['swivel'] = 0
        milestones = self.planToTransform(T_initial, link_index=EE_BASE_INDEX)
        self.addMilestones(milestones)
        self.plan.put(feasible=True)

    def pickToInspect(self, T_item):
        # Pick item
        self.pick(T_item)

        # Move to inspection station
        logger.debug('Moving to inspection station')
        self.setState('carrying')
        T_inspect = self.store.get('/robot/inspect_pose')
        milestones = self.planToTransform(T_inspect, link_index=EE_BASE_INDEX)
        self.addMilestones(milestones)
        self.plan.put(feasible=True)

    def stow(self, T_stow):
        if isinstance(T_stow, np.ndarray):
            T_stow = numpy2klampt(T_stow)
        self.checkCurrentConfig()

        # Move over stow location
        logger.debug('Moving over placement')
        state = 'carrying'
        self.setState(state)
        clearance_height = self.options['states'][state]['clearance_height']
        T_initial = (T_stow[0], [T_stow[1][0], T_stow[1][1], clearance_height])
        milestones = self.planToTransform(T_initial, link_index=EE_BASE_INDEX, name='stow_above')
        self.addMilestones(milestones)

        # Lower ee
        logger.debug('Descending to placement')
        self.setState('stowing_approach')
        milestones = self.planToTransform(T_stow, link_index=EE_BASE_INDEX, name='stow')
        self.addMilestones(milestones)

        # Place
        logger.debug('Placing')
        state = 'stowing'
        self.setState(state)
        delay = self.options['states'][state]['delay']
        vacuum = self.options['states'][state]['vacuum']
        self.addMilestone(Milestone(t=delay, robot_gripper=self.getCurrentConfig(), vacuum=vacuum))

        # Raise ee
        logger.debug('Ascending back over placement')
        self.setState('stowing_retraction')
        milestones = self.planToTransform(T_initial, link_index=EE_BASE_INDEX)
        self.addMilestones(milestones)
        self.plan.put(feasible=True)

    def planToTransform(self, T, link_index=None, name=None):
        if self.options['debug'] and name:
            self.store.put(['vantage', name], klampt2numpy(T))

        # Get inputs
        state = self.options['current_state']
        space = self.options['states'][state]['planning_space']
        solvers = self.options['states'][state]['planning_solvers']

        # Choose planner
        if space == 'joint':
            planner = self.joint_planner
        elif space == 'task':
            planner = self.task_planner
        else:
            raise RuntimeError('Unrecognized space: {}'.format(space))

        # Plan
        for i, solver in enumerate(solvers):
            self.options['current_solver'] = solver
            if space == 'task' and solver != 'nearby':
                logger.warn('Task space with {} solver requested. Intentional?'.format(solver))
            milestones = planner.planToTransform(T, link_index=link_index)
            # HACK: try again, assuming wrist flipped
            if milestones is None and space == 'task' and solver == 'nearby':
                T_failed = self.options['failed_pose']
                if T_failed is not None:
                    milestones = self._fixWristFlip(T, T_failed)
                    self.options['failed_pose'] = None
            # Update milestones, if found
            if milestones is not None:
                return milestones
            # Return, if all solvers failed
            elif i == len(solvers) - 1:
                logger.error('Infeasible Goal Transform: {}'.format(T))
                self.store.put('vantage/infeasible', klampt2numpy(T))
                return None
            else:
                logger.warning(
                    'IK {} solver failed. Trying {} solver...'.format(solver, solvers[i + 1]))

    def planToConfig(self, q):
        # Get inputs
        state = self.options['current_state']
        space = self.options['states'][state]['planning_space']
        solvers = self.options['states'][state]['planning_solvers']

        # Choose planner
        if space == 'joint':
            planner = self.joint_planner
        elif space == 'task':
            planner = self.task_planner
        else:
            raise RuntimeError('Unrecognized space: {}'.format(space))

        # Plan
        for i, solver in enumerate(solvers):
            self.options['current_solver'] = solver
            milestones = planner.planToConfig(q)
            # Update milestones, if found
            if milestones is not None:
                return milestones
            # Exit, if all solvers failed
            elif i == len(solvers) - 1:
                logger.error(
                    'Infeasible Goal Configuration: {}'.format(q))
                return None
            else:
                logger.warning(
                    'IK {} solver failed. Trying {} solver...'.format(solver, solvers[i + 1]))

    def _getInitialPickPose(self, T_item):
        state = self.options['current_state']
        clearance_height = self.options['states'][state]['clearance_height']
        R,t = T_item
        Rz = atan2(t[1], t[0]) - pi
        R_ee = numpy2klampt(xyz(*t) * rpy(pi, 0, 0) * rpy(0, 0, -Rz))[0]
        t_ee = (t[0], t[1], clearance_height)
        return (R_ee, t_ee)

    def _getRotationMatchingAxis(self, R, Rmatch, axis='x'):
        i = 0 if axis == 'x' else 1 if axis == 'y' else 2
        axis = [0] * 3
        axis[i] = 1
        v = so3.apply(R, axis)
        vm = so3.apply(Rmatch, axis)
        Rf = so3.vector_rotation(v, vm)
        return so3.mul(Rf, R)

    def _solveForPickConfig(self, T_item_normal, solvers=['local', 'global']):
        ee_base_link = self.robot.link(EE_BASE_INDEX)
        swivel_link = self.robot.link(SWIVEL_INDEX)
        T_item_no_normal = (self._getRotationMatchingAxis(T_item_normal[0], so3.identity(), axis='z'), T_item_normal[1])

        searchAngle = self.options['states']['picking']['search_angle_increment']
        d = so3.distance(T_item_normal[0], T_item_no_normal[0])
        numAttempts = 2 + ceil(d/radians(searchAngle))
        T_item_attempts = [se3.interpolate(T_item_normal, T_item_no_normal, u) for u in np.linspace(0, 1, numAttempts)]

        for i, T_item in enumerate(T_item_attempts):
            logger.debug('Trying normal interpolation {} of {}'.format(i + 1, numAttempts))

            # Create goals
            goals = []
            vacuum_tip_local = self.options['swivel_to_vacuum_tip']#[0.0025142, 0.0195, 0.05526]
            swivel_centered_local = vacuum_tip_local[:-1] + [0]

            # Match vacuum tip to item, and z axes
            local1 = [vacuum_tip_local, swivel_centered_local]
            world1 = [T_item[1], se3.apply(T_item, [0,0,vacuum_tip_local[2]])]
            goals.append(self.task_planner.getGoal(local=local1, world=world1, link_index=SWIVEL_INDEX))

            # Keep base of ee vertical, matching z axis to world
            T_swivel2ee = swivel_link.getParentTransform()
            swivel_centered_local = se3.apply(T_swivel2ee, swivel_centered_local)
            swivel_centered_world = se3.apply(T_item, [0,0,vacuum_tip_local[2]])
            ee_base_local = [0,0,0]
            ee_base_world = vops.add(swivel_centered_world, swivel_centered_local)
            local2 = [ee_base_local, swivel_centered_local]
            world2 = [ee_base_world, swivel_centered_world]
            goals.append(self.task_planner.getGoal(local=local2, world=world2, link_index=EE_BASE_INDEX))

            # Solve, trying all solvers
            q, i = None, 0
            while q is None and i < len(solvers):
                self.options['current_solver'] = solvers[i]
                q = self.task_planner.solve(goals)
                i +=1
            if q is not None and self.cspace.feasible(q):
                self.robot.setConfig(q)
                T_ee = self.robot.link(EE_BASE_INDEX).getTransform()
                T_swivel = self.robot.link(SWIVEL_INDEX).getTransform()
                # self.store.put('/vantage/pick_calc', klampt2numpy(T_swivel))
                return q, T_ee, T_swivel
        logger.error('No feasible configuration for item')
        self.plan.put(feasible=False)
        raise PlannerFailure('No feasible pick configuration for item')

    def _fixWristFlip(self, T, T_failed):
        logger.warn('Detected possible wrist flip. Trying hack...')
        self.options['current_solver'] = 'nearby'

        # Plan until failed T
        plan = MotionPlan(self.cspace, store=self.store)
        Ti_mid = list(numpy2klampt(klampt2numpy(T_failed) * rpy(0, 0, pi)))  # flip z
        milestones = self.task_planner.planToTransform(Ti_mid)
        success = plan.addMilestones(milestones, strict=False)
        if not success: return None

        # Plan until final T
        milestones = self.task_planner.planToTransform(T)
        success = plan.addMilestones(milestones, strict=False)
        if not success: return None
        logger.debug('Hack succeeded')
        return plan.milestones


class LowLevelPlanner(object):
    """
    Low level planner that solves for configurations and creates motion plans
    """

    def __init__(self, world, store=None, options=None):
        self.store = store or PensiveClient().default()
        self.world = world
        self.robot = self.world.robot('tx90l')
        self.options = options

    def reset(self, options):
        self.options = options

    def getGoal(self, T=None, local=None, world=None, link_index=None):
        # Get link
        link_index = link_index or EE_BASE_INDEX
        link = self.robot.link(link_index)

        # Create goal
        if T is not None:
            if isinstance(T, np.ndarray):
                T = numpy2klampt(T)
            goal = ik.objective(link, R=T[0], t=T[1])
        elif local is not None and world is not None:
            goal = ik.objective(link, local=local, world=world)
        else:
            raise RuntimeError('Must specify T, or local and world')
        return goal

    def solveForConfig(self, T=None, local=None, world=None, link_index=None, ):
        """Solves for desired configuration to reach the specified end effector transform"""
        goal = self.getGoal(T=T, local=local, world=world, link_index=link_index)
        return self.solve(goal)

    def solve(self, goals):
        solver = self.options['current_solver']
        q0 = self.options['current_config']
        eps = self.options['nearby_solver_tolerance']
        # self.robot.setConfig(q0)

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
            # logger.warning('IK {} solver failed'.format(solver))
            self.robot.setConfig(q0)
            return None


class JointPlanner(LowLevelPlanner):
    """
    Planner that works in joint (configuration) space
    """

    def __init__(self, space, store=None, options=None):
        super(JointPlanner, self).__init__(space.world, store=store, options=options)
        self.reset(space, options)

    def reset(self, cspace, options):
        super(JointPlanner, self).reset(options)
        self.space = cspace
        self.options = options

    def planToTransform(self, T, link_index=None):
        q = self.solveForConfig(T, link_index=link_index)
        if q is None:
            return None
        else:
            return self.planToConfig(q)

    def planToConfig(self, q):
        # Get inputs
        state = self.options['current_state']
        q0 = self.options['current_config']
        freq = self.options['control_frequency']
        profile = self.options['velocity_profile']
        vmax = self.options['states'][state]['joint_velocity_limits']
        amax = self.options['states'][state]['joint_acceleration_limits']
        vacuum = self.options['states'][state]['vacuum']
        swivel = self.options['swivel']

        # Calculate duration
        self.robot.setConfig(q0)
        q = self.space.getGeodesic(q0, q)
        if swivel is not None:
            q[SWIVEL_INDEX] = swivel
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
            m = Milestone(t=dt, robot_gripper=qi, vacuum=vacuum)
            milestones.append(m)
        return milestones


class TaskPlanner(LowLevelPlanner):
    """
    Planner that works in task (se3) space
    """

    def __init__(self, se3space, store=None, options=None):
        super(TaskPlanner, self).__init__(se3space.world, store=store, options=options)
        self.reset(se3space, options)

    def reset(self, se3space, options):
        super(TaskPlanner, self).reset(options)
        self.space = se3space
        self.options = options
        # self.vmax = 0.15
        # self.t_vmax, self.t_amax = 0.15, 0.15
        # self.R_vmax, self.t_amax = pi / 4, pi / 4

    def planToTransform(self, T, link_index=None):
        # Get inputs
        state = self.options['current_state']
        q0 = self.options['current_config']
        vmax = self.options['states'][state]['translation_velocity_limit']
        freq = self.options['control_frequency']
        profile = self.options['velocity_profile']
        vacuum = self.options['states'][state]['vacuum']
        swivel = self.options['swivel']

        if isinstance(T, np.ndarray):
            T = numpy2klampt(T)

        # Calculate duration
        link_index = link_index or EE_BASE_INDEX
        self.robot.setConfig(q0)
        T0 = self.robot.link(link_index).getTransform()
        # tf = self.space.getMinPathTime(T0, T, profile=self.profile)
        tf = vops.distance(T0[1], T[1]) / vmax
        dt = 1 / float(freq)
        if tf != 0:
            tf += abs(tf % dt - dt)     # make multiple of dt
            tf = max(tf, 1)             # enforce min time for ramp up/down

        # Add milestones
        milestones = []
        numMilestones = int(ceil(tf * freq))
        for t in np.linspace(dt, tf, numMilestones):
            Ti = self.space.interpolate(T0, T, t, tf, profile=profile)
            q = self.solveForConfig(Ti, link_index=link_index)
            if q is None:
                self.robot.setConfig(q)
                self.options['failed_pose'] = Ti
                # self.store.put('/vantage/failed', klampt2numpy(Ti))
                return None
            # Add milestone
            if swivel is not None:
                q[SWIVEL_INDEX] = q0[SWIVEL_INDEX] * (1-t/tf) + swivel*(t/tf)
            m = Milestone(t=dt, robot_gripper=q, vacuum=vacuum)
            milestones.append(m)
        return milestones

    def planToConfig(self, q, link_index=None):
        # q = self.cspace.getGeodesic(q0, q)
        link_index = link_index or EE_BASE_INDEX
        self.robot.setConfig(q)
        Tf =self.robot.link(link_index).getTransform()
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
        self.failed_milestones = []

    def feasible(self):
        for milestone in self.milestones:
            if not self.cspace.feasible(milestone.get_robot_gripper()):
                return False
        return True

    def getFeasibleMilestones(self):
        feasible_milestones = []
        for milestone in self.milestones:
            if self.cspace.feasible(milestone.get_robot_gripper()):
                feasible_milestones.append(milestone)
            else:
                break
        return feasible_milestones

    def getCurrentConfig(self):
        if len(self.milestones) != 0:
            return self.milestones[-1].get_robot_gripper()
        return None

    def addMilestone(self, milestone, strict=True):
        # Append milestone, only if feasible
        if milestone is not None:
            if self.cspace.feasible(milestone.get_robot_gripper()):
                self.milestones.append(milestone)
                return True
        # Raise error if strict
        if strict:
            self.failed_milestones = self.milestones[:]
            self.put(feasible=False)
            raise PlannerFailure('infeasible milestone')
        # Otherwise, return failed
        else:
            return False

    def addMilestones(self, milestones, strict=True):
        # Append milestones, only if feasible
        if milestones is None:
            if strict:
                self.put(feasible=False)
                raise PlannerFailure('infeasible milestone')
            else:
                return False
        else:
            for i, milestone in enumerate(milestones):
                if not self.addMilestone(milestone):
                    del self.milestones[-i:]
                    l1 = len(self.milestones)
                    if strict:
                        self.put(feasible=False)
                        raise PlannerFailure('infeasible milestone')
                    else:
                        return False
        return True

    def put(self, feasible=None):
        # Check feasibility
        if feasible is None:
            feasible = self.feasible()

        # Update database
        t = time()
        if feasible:
            logger.info('Feasible path found. Updating waypoints...')
            milestoneMap = [m.get_milestone() for m in self.milestones]
            self.store.put('/robot/waypoints', milestoneMap)
            self.store.put('/robot/timestamp', t)
        else:
            logger.error('Failed to find feasible path. Clearing waypoints...')
            # Clear waypoints
            self.store.put('/robot/waypoints', None)
            self.store.put('/robot/timestamp', t)
            # Update debug waypoints
            milestoneMap = [m.get_milestone() for m in self.failed_milestones]
            self.store.put('/debug/waypoints', milestoneMap)
            self.store.put('/debug/timestamp', t)


class CSpace:
    """
    A configuration space used to determine feasible configurations
    """

    def __init__(self, world, store=None):
        self.store = store or PensiveClient().default()
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
        # elif profile == 'trapeze':
        #     vmax = vmax or self.robot.getVelocityLimits()
        #     amax = amax or self.robot.getAccelerationLimits()
        #     q = q0[:]
        #     for i, (q0i, qfi, vi, ai) in enumerate(zip(q0, qf, vmax, amax)):
        #         tau = vi / ai if ai != 0 else 0
        #         Di = qfi - q0i
        #         Dsign = np.sign(Di)
        #         if 0 <= t < tau:
        #             q[i] = q0i + 0.5 * t**2 * ai * Dsign
        #         elif tau <= t < tf - tau:
        #             q[i] = q0i + (t - tau / 2) * vi * Dsign
        #         else:
        #             q[i] = qfi - 0.5 * (tf - t)**2 * ai * Dsign
        #     return q
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
            # elif profile == 'trapeze':
            #     tf.append(vi / ai + D / vi if vi and ai != 0 else 0)
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
                    'Joint Limit Error: joint {} not in [{}, {}]'.format(i, qli, qui))
                self.store.put('planner/failure', 'exceeded_joint_limits')
                return False
        return True

    def noSelfCollision(self, q):
        """Checks for self collisions at q"""
        self.robot.setConfig(q)
        if self.robot.selfCollides():
            logger.error('Collision Error: robot colliding with self')
            self.store.put('planner/failure', 'self_collision')
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
                self.store.put('planner/failure', 'environment_collision')
                return False
        # Check terrain collisions
        for o in xrange(self.world.numTerrains()):
            if any(self.collider.robotTerrainCollisions(r, o)):
                name = self.world.terrain(o).getName()
                logger.error(
                    'Collision Error: robot colliding with {}'.format(name))
                self.store.put('planner/failure', 'environment_collision')
                return False
        return True


class SE3Space:
    """
    An SE3 space used to plan cartesian movements
    """

    def __init__(self, world, store=None):
        self.store = store or PensiveClient().default()
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

def sph2cart(az, el, r):
    a = r * np.cos(el)
    x,y,z = a*np.cos(az), a*np.sin(az), r*np.sin(el)
    return (x,y,z)

def test_pick(location, rand_seed=None, buffer=0.1, store=None, planner=None):
    logger.debug('Testing pick for {} with seed {}'.format(location,rand_seed))

    # Declare
    seed(rand_seed)
    store = store or PensiveClient().default()
    planner = planner or MotionPlanner(store=store)

    # Calculate random rotation
    theta, phi = acos(2*uniform(0,1) - 1), 2*pi*uniform(0,1)
    axis, angle = sph2cart(theta, phi, 1), uniform(0,pi/2)
    R = so3.from_axis_angle((axis,angle))

    # Calculate random translation
    if location.startswith('bin'):
        T_ref = numpy2klampt(store.get('/shelf/pose'))
        bounds = store.get('/shelf/bin/{}/bounds'.format(location), strict=True)
    elif location.startswith('box'):
        T_ref = numpy2klampt(store.get('/box/{}/pose'.format(location), strict=True))
        bounds = store.get('/box/{}/bounds'.format(location), strict=True)
    else:
        raise RuntimeError('Unrecognized location: "{}"'.format(location))

    # Plan
    t = [uniform(a+buffer,b-buffer) for a,b in zip(*bounds)]
    T_item = (R, se3.apply(T_ref, t)) # se3.mul(T_ref, (R,t))
    store.put('/vantage/item', klampt2numpy(T_item))
    planner.pick(T_item)

if __name__ == "__main__":
    from random import seed, uniform, randint, choice
    store = PensiveClient().default()
    planner = MotionPlanner(store=store)
    bins = store.get('/shelf/bin').keys()
    boxes = store.get('/box').keys()

    tests = 100
    passes = tests

    t = time()
    for i in range(tests):
        try:
            planner.reset()
            test_pick(choice(bins + boxes), rand_seed=randint(0,1000), store=store, planner=planner)
        except PlannerFailure:
            assert(store.get('robot/waypoints') is None)
            passes -= 1
        else:
            planner.robot.setConfig(planner.plan.milestones[-1].get_robot_gripper())
            assert(planner.plan.feasible())

    logger.info('Planned {} routes in {} s'.format(tests, time()-t))
    logger.info('Success rate: {} %'.format(passes/float(tests)*100))
