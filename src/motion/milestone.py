from klampt.math import vectorops

import math
import logging; logger = logging.getLogger(__name__)

DOF = { 'db': 7,
        'robot': 6,
        'vacuum': 1,
        'gripper': 1
}

class Milestone:
    def __init__(self, t=None, robot=None, vacuum=None, gripper=None, map=None, type='db'):
        # Set values
        self.type = type.lower()
        self.t = t or 1
        self.robot = robot or [0]*DOF[self.type]
        self.gripper = gripper or [0]*DOF['gripper']
        self.vacuum = vacuum or [0]*DOF['vacuum']
        # Set values from map, if applicable
        if map is not None:
            self.set_milestone(map)
        # Validate
        self.check()

    def check(self):
        # Check type
        if self.type not in ['robot', 'db']:
            raise RuntimeError('Unrecognized milestone type: "{}"'.format(self.type))
        # Check dofs
        if DOF[self.type] != len(self.robot):
            raise RuntimeError('Milestone type and robot dof mismatch: "{}: {} != {}"'.format(self.type, DOF[self.type], len(self.robot)))
        if DOF['gripper'] != len(self.gripper):
            raise RuntimeError('Gripper dof mismatch: "{} != {}"'.format(DOF['gripper'], len(self.gripper)))
        if DOF['vacuum'] != len(self.vacuum):
            raise RuntimeError('Vacuum dof mismatch: "{} != {}"'.format(DOF['vacuum'], len(self.vacuum)))
        # Check robot configs are numbers
        for qi in self.robot:
            if math.isinf(qi) or math.isnan(qi):
                raise RuntimeError('Invalid configuration {}'.format(qi))

    def get_milestone(self):
        return (self.t, {
                  'robot': self.robot,
                  'gripper': self.gripper,
                  'vacuum': self.vacuum,
                })

    def set_milestone(self, milestone):
        self.t = milestone[0]
        self.robot = milestone[1]['robot']
        self.gripper = milestone[1]['gripper']
        self.vacuum = milestone[1]['vacuum']

    def get_type(self):
        return self.type

    def set_type(self, newType):
        # Check type
        logger.info('From {} to {}'.format(self.type, newType))
        newType = newType.lower()
        if newType not in ['robot', 'db']:
            raise Exception('Unrecognized option: "{}"'.format(type))
        # Convert
        if self.type == 'db' and newType == 'robot':
            # To degrees, ignoring extraneous q0
            self.robot = [math.degrees(qi) for qi in self.robot][1:]
            # Flip joints
            self.robot[2] *= -1
            self.robot[4] *= -1
        elif self.type =='robot' and newType == 'db':
            # Flip joints
            self.robot[2] *= -1
            self.robot[4] *= -1
            # To radians, adding extraneous q0
            self.robot = [0] + [math.radians(qi) for qi in self.robot]
        elif self.type == newType:
            pass
        self.type = newType
        self.check()

    #def set_milestone(self, milestone):
    #    self.milestone = milestone

    def scale_t(self, scale):
        # Check scale
        if not (0 < scale <= 1):
            raise RuntimeError('Speed scale "{}" is not in (0,1]'.format(scale))
        # Apply scale
        self.t /= float(scale)

    def get_t(self):
        return self.t

    def set_t(self, t):
        self.t = t

    def get_robot(self):
        return self.robot

    def set_robot(self, robot):
        self.robot = robot

    def get_gripper(self):
        return self.gripper

    def set_gripper(self, gripper):
        self.gripper = gripper

    def get_vacuum(self):
        return self.vacuum

    def set_vacuum(self, vacuum):
        self.vacuum = vacuum

    #Fixes an array of milestones
    def fix_milestones(self, motion_milestones):
        max_change=5.0/180.0*3.14159
        max_speed=60/180.0*3.14159
        old_config=motion_milestones[0].get_robot()
        i=1
        while i<len(motion_milestones):
            new_config=motion_milestones[i].get_robot()
            d_config=max(max(vectorops.sub(new_config,old_config)),-min(vectorops.sub(new_config,old_config)))
            speed_config=d_config/motion_milestones[i].get_t()
            if d_config>max_change:
                new_milestone=Milestone(motion_milestones[i].get_t(),vectorops.div(vectorops.add(motion_milestones[i-1].get_robot(),motion_milestones[i].get_robot()),2),motion_milestones[i-1].get_vacuum())
                motion_milestones.insert(i,new_milestone)
                continue
            elif speed_config>max_speed:
                new_milestone=Milestone(d_config/(max_speed-0.1),motion_milestones[i].get_robot(),motion_milestones[i].get_vacuum())
                motion_milestones[i]=new_milestone
                i+=1
                old_config=new_config
            else:
                i+=1
                old_config=new_config
        return motion_milestones
