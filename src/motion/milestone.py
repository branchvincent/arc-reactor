from klampt import *
from klampt.vis.glrobotprogram import *

class Milestone():
    def __init__(self, t=None, q=None, vacuum_status=None):
        self.t = t
        self.robot = q
        self.gripper = [0,0,0]
        self.vacuum = vacuum_status

    def get_milestone(self):
        return (self.t, {
                  'robot': self.robot,
                  'gripper': self.gripper,
                  'vacuum': self.vacuum,
                })
    
    #def set_milestone(self, milestone):
    #    self.milestone = milestone

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

