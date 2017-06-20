import sys
from klampt import *
from klampt.vis.glrobotprogram import *
from klampt.model import ik,coordinates,config,cartesian_trajectory,trajectory
from klampt.model.collide import WorldCollider
from klampt.model import collide
from klampt.plan.cspace import CSpace,MotionPlan
from klampt.math import so3,se3
import random
import importlib
import math
import os
import time
import json
import copy
from pensive.client import PensiveClient
from motion.milestone import Milestone


class PickPlanner():

    def __init__(self, world, store):
        self.world = world
        self.robot=self.world.robot(0)
        self.check_points=[]
        self.motion_milestones=[]
        self.store = store or PensiveClient().default()

        #get necessary constants from the db

        self.ee_local = self.store.get('/planner/ee_local', [0.0, 0.0, 0.4])
        self.ee_link = self.store.get('/planner/ee_link', 6)        
        self.control_rate= self.store.get('/planner/control_rate', 20) #controlling the self.robot with 20 Hz
        self.max_end_effector_v = self.store.get('/planner/max_ee_v', 0.15) # max end effector move speed
        self.max_change = self.store.get('/planner/max_milestone_change', 0.35367795) # 200.0/180.0*3.14159 
                #the max change between raw milestones is 20 degree
        self.slow_down_factor = self.store.get('/planner/slow_down', 0.4) 
                #should be (0,1), for when robot is holding item
        self.vacuum_approach_dist = self.store.get('/planner/vac_approach_d', [0, 0, 0.03])
                #should this be different for pick/stow, get/drop?

        #unused...? trash if confirmed unused

        #self.box_release_offset = self.store.get('/planner/release_offset', [0,0,0.06])
        #self.order_box_min = self.store.get('/planner/box_min', [0.36,0.65,0.5])
        #self.order_box_max = self.store.get('/planner/box_max', [0.5278,0.904,0.5])
        #self.angle_to_degree = 57.296

        
    def clear_milestones(self):
        self.motion_milestones=[]
    
    def pick_up(self, item):
        """
        This function will return a motion plan that will pick up the target item from the shelf.
        Inputs:
            - world model: including klampt models for the self.robot, the shelf and the target container
            - item: position/orientation of the target item
                -- position: item position
                -- vacuum_offset: hacked parameter for each item, added to the high of the item
        
        Outputs:
            a list of Milestones
            check_points for the motion plan:
                -- a list of transform matrixs for the end-effector at key points for the plan
        """
        #setup initial path to above item (in shelf)
        self.current_config=self.robot.getConfig()
        curr_position=self.robot.link(self.ee_link).getWorldPosition(self.ee_local)
        curr_orientation, p = self.robot.link(self.ee_link).getTransform()
        current_T=[curr_orientation,curr_position]
        
        item_position=vectorops.div(vectorops.add(item['bbox'][0],item['bbox'][1]),2.0)
        item_vacuum_offset=item['vacuum_offset']
        
        test_cspace=TestCSpace(Globals(self.world))

        self.check_points.append(current_T)
        self.motion_milestones=self.joint_space_rotate(self.motion_milestones,p,item_position,self.robot,0)

        if self.motion_milestones is None:
            raise RuntimeError("First motion milestone is non-existent")

        #move the self.robot from current position to a start position that is above the target item
        curr_position=self.robot.link(self.ee_link).getWorldPosition(self.ee_local)
        curr_orientation,p=self.robot.link(self.ee_link).getTransform()
        current_T=[curr_orientation,curr_position]
        self.check_points.append(current_T)
        
        start_position=vectorops.add(item_position,[0,0,0.4])
        start_position[2]=min(0.4,start_position[2])
        end_T=[[1,0,0,0,-1,0,0,0,-1],start_position]
        self.check_points.append(end_T)
        l=vectorops.distance(current_T[1],end_T[1])

        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,current_T,end_T,0,0,1)

        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to start position")

        #TODO insert break here for separating states when TCP camera works

        #start the vacuum
        self.motion_milestones.append(Milestone(1,self.robot.getConfig(),1))

        #lower the vacuum
        start_T=copy.deepcopy(end_T) 
        end_T[1]=vectorops.add(item_position,item_vacuum_offset)
        self.check_points.append(end_T)
        l=vectorops.distance(start_T[1],end_T[1])

        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,1,0,1)

        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to lower the vacuum")

        #pick up the item
        start_T=copy.deepcopy(end_T)
        end_T[1][2]+=0.4
        l=vectorops.distance(start_T[1],end_T[1])
        self.check_points.append(end_T)

        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,1,1,1)

        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to pick up item")    

        #find and move to the inspection station
        inspection_pose = self.store.get('/robot/inspect_pose');
        inspect_position = inspection_pose[:3,3]

        curr_orientation,p=self.robot.link(self.ee_link).getTransform()
        self.motion_milestones=self.joint_space_rotate(self.motion_milestones,p,inspect_position,self.robot,1)

        curr_position=self.robot.link(self.ee_link).getWorldPosition(self.ee_local)
        curr_orientation,p=self.robot.link(self.ee_link).getTransform()
        current_T=[curr_orientation,curr_position]

        start_T=copy.deepcopy(current_T)
        end_T=copy.deepcopy(current_T)
        end_T[1][0]=inspect_position[0]
        end_T[1][1]=inspect_position[1]
        self.check_points.append(end_T)
        l=vectorops.distance(start_T[1],end_T[1])

        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,1,1,1)

        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to move item to next location")

        #TODO code to return incomplete array for debugging
        print "Returning motion plan to inspection station"
        return self.motion_milestones

    def drop_item(self, item, target_box, target_index):
        #initial setup
        self.current_config=self.robot.getConfig()
        curr_position=self.robot.link(self.ee_link).getWorldPosition(self.ee_local)
        curr_orientation, p = self.robot.link(self.ee_link).getTransform()
        current_T=[curr_orientation,curr_position]
        test_cspace=TestCSpace(Globals(self.world))
        self.check_points.append(current_T)

        #get end point
        drop_position = self.find_placement(target_box, target_index)
        self.motion_milestones=self.joint_space_rotate(self.motion_milestones,p,drop_position,self.robot,1)

        curr_position=self.robot.link(self.ee_link).getWorldPosition(self.ee_local)
        curr_orientation,p=self.robot.link(self.ee_link).getTransform()
        current_T=[curr_orientation,curr_position]
        self.check_points.append(current_T)
        
        item_vacuum_offset=item['vacuum_offset']
        drop_offset=item['drop offset']

        start_position=vectorops.add(drop_position,[0,0,0.4])
        start_position[2]=min(0.4,start_position[2])
        end_T=[[1,0,0,0,-1,0,0,0,-1],start_position]
        self.check_points.append(end_T)
        l=vectorops.distance(current_T[1],end_T[1])

        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,current_T,end_T,0,0,1)

        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to start position")

        #move to box location


        #lower the item
        start_T=copy.deepcopy(end_T)
        end_T[1]=vectorops.add(drop_position,drop_offset)
        self.check_points.append(end_T)
        l=vectorops.distance(start_T[1],end_T[1])
        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,1,1,1)
        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to lower the vacuum/item")

        #turn off the vacuum
        self.motion_milestones.append(Milestone(1,self.robot.getConfig(),0))

        #raise the robot
        start_T=copy.deepcopy(end_T)
        end_T[1][2]=0.45
        self.check_points.append(end_T)
        l=vectorops.distance(start_T[1],end_T[1])
        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,0,0,0)
        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to raise the robot")

        self.motion_milestones = Milestone().fix_milestones(self.motion_milestones)
        return self.motion_milestones

    def find_placement(self, target_box, target_index):
        print "target index is ", target_index
        #figure out spot to put item. current random?
        box_min,box_max=target_box["box_limit"]
        box_T = self.world.rigidObject('{}_box'.format(target_box["name"])).getTransform()
        item_T = self.world.rigidObject(target_index).getTransform()
        goal_T=[[],[]]
        #goal_T[0]=item_T[0]
        flag=1
        n=0
        while (flag and n<20):
            x=random.uniform(box_min[0]+0.1,box_max[0]-0.1)
            y=random.uniform(box_min[1]+0.1,box_max[1]-0.1)
            z=random.uniform(box_min[2]+0.1,box_max[2]-0.1)
            goal_T[1] = se3.apply(box_T, [x, y, z])
            goal_T[0] = item_T[0]
            print "goal T ", goal_T[1]
            print "box ", box_T[1]
            if self.check_placement(self.world,goal_T,target_index):
                flag=0
                drop_position=goal_T[1]
                box_bottom_high=goal_T[1][2]
            n+=1
        if flag:
            raise RuntimeError("here is the error")
            print "can't find a feasible placement in the box for the item"
            self.world.rigidObject(target_index).setTransform(item_T[0],item_T[1])
            return False
       # n=0
       # check_flag=1
       # low_bound=0
       # high_bound=z
       # while n<5:
       #     new_z=0.5*(low_bound+high_bound)
       #     goal_T[1]=[x,y,new_z]
       #     if self.check_placement(self.world,goal_T,target_index):
       #         drop_position=goal_T[1]
       #         box_bottom_high=goal_T[1][2]
       #         high_bound=new_z
                # print 'lower!!'
       #     else:
       #         low_bound=0.5*(low_bound+high_bound)
       #     n+=1
        print 'find a placement:',goal_T[1]
        self.world.rigidObject(target_index).setTransform(item_T[0],item_T[1])
        return drop_position

    def joint_space_rotate(self,motion_milestones,current_p,target_p,robot,vacuum_status):
        theta1=math.atan2(current_p[1],current_p[0])
        theta2=math.atan2(target_p[1],target_p[0])
        delta=theta2-theta1
        if delta>math.pi:
            delta=delta-2*math.pi
        elif delta<-math.pi:
            delta=delta+2*math.pi
        max_rotate_speed=0.5*(1-self.slow_down_factor*vacuum_status)
        t_step=1.0/self.control_rate
        motion_milestones.append(Milestone(0.05,self.robot.getConfig(),vacuum_status))
        if delta>0:
            #ccw rotate
            total_t=delta/max_rotate_speed
            steps=int(total_t/t_step)+1
            t=0
            old_q=0
            new_q=0
            while t<=steps:
                x=t*1.0/steps
                new_q=(10*math.pow(x,3)-15*math.pow(x,4)+6*math.pow(x,5))*delta
                dq=new_q-old_q
                q=self.robot.getConfig()
                q[1]+=dq
                q[6]+=dq
                old_q=new_q
                motion_milestones.append(Milestone(t_step,q,vacuum_status))
                self.robot.setConfig(q)
                t+=1
        else:
            total_t=-delta/max_rotate_speed
            steps=int(total_t/t_step)+1
            t=0
            old_q=0
            new_q=0
            while t<=steps:
                x=t*1.0/steps
                new_q=-(10*math.pow(x,3)-15*math.pow(x,4)+6*math.pow(x,5))*delta
                dq=new_q-old_q
                q=self.robot.getConfig()
                q[1]-=dq
                q[6]-=dq
                old_q=new_q
                motion_milestones.append(Milestone(t_step,q,vacuum_status))
                self.robot.setConfig(q)
                t+=1
        return motion_milestones

    def check_placement(self,world,T,target_index):
#only for item placement checking
        world.rigidObject(target_index).setTransform(T[0],T[1])
        glist_target=[]
        glist_target.append(world.rigidObject(target_index).geometry())
        glist_terrain=[]
        badStuff = []
        glist_object=[]
        for i in xrange(world.numTerrains()):
            t = world.terrain(i)
            g = t.geometry()
            if g != None and g.type()!="":
                glist_terrain.append(g)
        for i in xrange(world.numRigidObjects()):
            o = world.rigidObject(i)
            g = o.geometry()
            if g != None and g.type()!="" and o.getName()!=target_index:
                glist_object.append(g)
                badStuff.append(world.rigidObject(i).getName())
        listMe = list(collide.group_collision_iter(glist_target,glist_object))
        if any(listMe):
        # if any(collide.self_collision_iter(glist_target+glist_object)):
            print 'bad object: ', badStuff[listMe[0][1]]
            print 'objects colliding!'
            return False
        if any(collide.group_collision_iter(glist_target,glist_terrain)):
        # if any(collide.self_collision_iter(glist_target+glist_terrain)):
            print 'terrain colliding!'
            return False
        return True

    def add_milestones(self,test_cspace,robot,milestones,t,control_rate,start_T,end_T,vacuum_status,simulation_status,ik_indicator):
        if t<0.5:
            t=0.5
        t=t/(1-self.slow_down_factor*vacuum_status)
        steps=int(t*self.control_rate)+1
        t_step=1.0/self.control_rate
        # print "start config",self.robot.getConfig()
        i=0
        start_q=self.robot.getConfig()
        milestones.append(Milestone(0.05,start_q,vacuum_status))
        while i<=steps:
            q_old=self.robot.getConfig()
            x=i*1.0/steps
            u=10*math.pow(x,3)-15*math.pow(x,4)+6*math.pow(x,5)
            # print u
            [local_p,world_p]=self.interpolate(start_T,end_T,u,ik_indicator)
            goal = ik.objective(self.robot.link(self.ee_link),local=local_p,world=world_p)
            s=ik.solve_global(goal)
            # s=ik.solve_nearby(goal,maxDeviation=1000,feasibilityCheck=test_function)
            q=self.robot.getConfig()
            n=0
            flag = 1
            if (max(vectorops.sub(q_old,q))>self.max_change) or (min(vectorops.sub(q_old,q))<(-self.max_change)) or q[3]*start_q[3]<0 <0:
                print "too much change!"
                print vectorops.sub(q_old,q)
                flag=0

            while n<30:
                if flag and (s and test_cspace.feasible(q)) :
                    break
                else:
                    # print "no feasible ik solution found"
                    # print world_p
                    self.robot.setConfig(q_old)
                    s=ik.solve_global(goal)
                    # s=ik.solve_nearby(goal,maxDeviation=1000,feasibilityCheck=test_function)
                    q=self.robot.getConfig()
                    if (max(vectorops.sub(q_old,q))>self.max_change) or (min(vectorops.sub(q_old,q))<(-self.max_change)) or q[3]*start_q[3]<0 :
                        print "too much change!"
                        flag=0
                    else:
                        flag=1
                    # print 's',s
                    # print 'feasible test:',test_cspace.feasible(q)
                    n+=1
            if flag and s and test_cspace.feasible(q):
                m_change=max(max(vectorops.sub(q_old,q)),-min(vectorops.sub(q_old,q)))
                milestones.append(Milestone(t_step,q,vacuum_status))
                i+=1
            else:
                print 'no feasible solution can be found!!!!!'
                print 'ik solver:',s
                print 'maximum change:',flag
                return False
        return milestones

    def interpolate(self,start_T,end_T,u,flag):
        R1,t1=start_T
        R2,t2=end_T
        T=se3.interpolate(start_T,end_T,u)
        R,t=T
        if flag:
            ik_local=[self.ee_local,vectorops.add(self.ee_local,[0.01,0,0]),vectorops.add(self.ee_local,[0,0,0.01])]
            ik_world=[t,vectorops.add(t,[R[0]/100.0,R[1]/100.0,R[2]/100.0]),vectorops.add(t,[R[6]/100.0,R[7]/100.0,R[8]/100.0])]
        else:
            ik_local=[self.ee_local,vectorops.add(self.ee_local,[0,0,0.01])]
            ik_world=[t,vectorops.add(t,[R[6]/100.0,R[7]/100.0,R[8]/100.0])]
        return [ik_local,ik_world]


class StowPlanner():
    def __init__(self, world, store):
        self.world = world
        self.robot=self.world.robot(0)
        self.check_points=[]
        self.motion_milestones=[]
        self.store = store or PensiveClient().default()

        #get necessary constants from the db
        self.ee_local = self.store.get('/planner/ee_local', [0.0, 0.0, 0.4])
        self.ee_link = self.store.get('/planner/ee_link', 6)        
        self.control_rate= self.store.get('/planner/control_rate', 20) #controlling the self.robot with 20 Hz
        self.max_end_effector_v = self.store.get('/planner/max_ee_v', 0.15) # max end effector move speed
        self.max_change = self.store.get('/planner/max_milestone_change', 0.35367795) # 200.0/180.0*3.14159 
                #the max change between raw milestones is 20 degree
        self.slow_down_factor = self.store.get('/planner/slow_down', 0.4) 
                #should be (0,1), for when robot is holding item
        self.vacuum_approach_dist = self.store.get('/planner/vac_approach_d', [0, 0, 0.03])
                #should this be different for pick/stow, get/drop?

    def clear_milestones(self):
        self.motion_milestones=[]

    def stow(world,item,target_box,target_index):
        """
        This function will return a motion plan that will pick up the target item from the tote and place it to the shelf.
        Inputs:
            - world model: including klampt models for the self.robot, the shelf and the target container
            - item: position/orientation of the target item, ideal surfaces and approach directions for vacuum or preferred grasp configurations for known item
                -- position: item position
                -- vacuum_offset: hacked parameter for each item, added to the high of the item to find the grasp position for the vacuum
                -- drop offset: hacked parameter for each item, added to the high of the order box bottom high to find the drop position for the vacuum
        outputs:
            a list of milestones=(t, {
                  'self.robot': q,
                  'gripper': [0,0,0],
                  'vaccum': [0]
                })
            - t: planned time to reach this configuration
            - self.robot q: raw output from the klampt simulation. q=[q0,q1,q2,q3,q4,q5,q6]. q0 is alwasy 0. The signs for q3 and q5 is flipped on the real self.robot.
            Also all the values are in radius, while the real self.robot joint values are in degree
            - gripper control
            - vaccum: 0-off 1-on
        """

        self.robot=world.self.robot(0)
        self.motion_milestones=[]
        current_config=self.robot.getConfig()
        curr_position=self.robot.link(self.ee_link).getWorldPosition(self.ee_local)
        curr_orientation,p=self.robot.link(self.ee_link).getTransform()
        current_T=[curr_orientation,curr_position]
        # item_position=item['position']
        item_position=vectorops.div(vectorops.add(item['bbox'][0],item['bbox'][1]),2.0)
        item_vacuum_offset=item['vacuum_offset']
        drop_offset=item['drop offset']

        vaccum_approach_distance=[0,0,0.15]

        test_cspace=TestCSpace(Globals(world))



    #TODO break out stowing position finding into separate function/state

        #find a stowing position if no goal position is given by the input
        if target_box['drop position']:
            drop_position=target_box['drop position']
            # print drop_position
            box_bottom_high=target_box['position'][2]
        else:
            # print "here!!!!!!!!"
            box_min,box_max=target_box["box_limit"]
            origin_T=world.rigidObject(target_index).getTransform()
            goal_T=[[],[]]
            goal_T[0]=origin_T[0]
            flag=1
            n=0
            while (flag and n<10):
                x=random.uniform(box_min[0],box_max[0])
                y=random.uniform(box_min[1],box_max[1])
                z=random.uniform(box_min[2],box_max[2])
                goal_T[1]=[x,y,z]
                # print goal_T[1]
                # goal_T[1]=[1.10298067096586, 0.3038671358694375, 0.791611841275841]
                if check_placement(world,goal_T,target_index):
                    flag=0
                    drop_position=goal_T[1]
                    box_bottom_high=goal_T[1][2]
                n+=1
            if flag:
                raise RuntimeError("Can't find a feasible placement")
                world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
    #            return False
            n=0
            check_flag=1
            low_bound=0
            high_bound=z
            while n<5:
                new_z=0.5*(low_bound+high_bound)
                goal_T[1]=[x,y,new_z]
                if check_placement(world,goal_T,target_index):
                    drop_position=goal_T[1]
                    box_bottom_high=goal_T[1][2]
                    high_bound=new_z
                    # print 'lower!!'
                else:
                    low_bound=0.5*(low_bound+high_bound)
                n+=1
            print 'find a placement:',goal_T[1]
            world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])




        self.motion_milestones=joint_space_rotate(self.motion_milestones,p,item_position,self.robot,0)
        curr_position=self.robot.link(self.ee_link).getWorldPosition(self.ee_local)
        curr_orientation,p=self.robot.link(self.ee_link).getTransform()
        current_T=[curr_orientation,curr_position]
        #move the self.robot from current position to a start position that is above the target item
        start_position=vectorops.add(item_position,[0,0,0.3])
        end_T=[[1,0,0,0,-1,0,0,0,-1],start_position]
        l=vectorops.distance(current_T[1],end_T[1])
        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,current_T,end_T,0,0,1)
        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to start position")
    #        return False

        #start the vacuum
        self.motion_milestones.append(Milestone(1,self.robot.getConfig(),1))

        #lower the vacuum
        start_T=copy.deepcopy(end_T)
        end_T[1]=vectorops.add(item_position,item_vacuum_offset)
        l=vectorops.distance(start_T[1],end_T[1])

        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,1,0,1)
        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to lower the vacuum")
     #       return False

        #pick up the item
        start_T=copy.deepcopy(end_T)
        end_T[1][2]+=0.4
        l=vectorops.distance(start_T[1],end_T[1])
        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,1,1,1)
        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to pick up the item")
       #     return False


        curr_orientation,p=self.robot.link(self.ee_link).getTransform()
        self.motion_milestones=joint_space_rotate(self.motion_milestones,p,drop_position,self.robot,1)
        # while drop_position[0]*p[1]>p[0]*drop_position[1]:
        #     q=self.robot.getConfig()
        #     q[1]-=0.05
        #     q[6]-=0.05
        #     self.motion_milestones.append(make_milestone(0.05,q,1,1))
        #     self.robot.setConfig(q)
        #     curr_orientation,p=self.robot.link(self.ee_link).getTransform()



        #move the item to the start position for dropping
        curr_position=self.robot.link(self.ee_link).getWorldPosition(self.ee_local)
        curr_orientation,p=self.robot.link(self.ee_link).getTransform()
        current_T=[curr_orientation,curr_position]

        start_T=copy.deepcopy(current_T)
        end_T=copy.deepcopy(current_T)
        end_T[1][0]=drop_position[0]
        end_T[1][1]=drop_position[1]
        l=vectorops.distance(start_T[1],end_T[1])
        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,1,1,1)
        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to drop position")
           # return False

        #lower the item
        start_T=copy.deepcopy(end_T)
        end_T[1]=vectorops.add(drop_position,drop_offset)
        l=vectorops.distance(start_T[1],end_T[1])
        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,1,1,1)
        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to lower the item")
            #return False

        #turn off the vacuum
        self.motion_milestones.append(Milestone(1,self.robot.getConfig(),0))

        #raise the self.robot
        start_T=copy.deepcopy(end_T)
        end_T[1][2]=0.45
        l=vectorops.distance(start_T[1],end_T[1])
        self.motion_milestones=self.add_milestones(test_cspace,self.robot,self.motion_milestones,l/self.max_end_effector_v,self.control_rate,start_T,end_T,0,0,0)
        if not self.motion_milestones:
            raise RuntimeError("Can't find a feasible path to raise the self.robot")
          #  return False


        Milestone.fix_milestones(self.motion_milestones)
        return self.motion_milestones


class Globals:
    def __init__(self,world):
        self.world = world
        self.robot = self.world.robot(0)
        self.collider = WorldCollider(self.world)

class TestCSpace(CSpace):
    """A CSpace defining the feasibility constraints of the self.robot"""
    def __init__(self,globals):
        CSpace.__init__(self)
        self.globals = globals
        self.robot = globals.robot
        #initial whole-body configuration
        self.q0 = self.robot.getConfig()
        #setup CSpace sampling range
        qlimits = zip(*self.robot.getJointLimits())
        self.bound = [qlimits[i] for i in range(len(self.q0))]
        #setup CSpace edge checking epsilon
        self.eps = 1e-2

    def feasible(self,x):
        #check arm joint limits
        for (xi,bi) in zip(x,self.bound):
            if xi < bi[0] or xi > bi[1]:
                return False
        #set up whole body configuration to test environment and self collisions
        q =x[:]
        self.robot.setConfig(q)
        world = self.globals.world
        collider = self.globals.collider
        #TODO: this could be much more efficient if you only tested the self.robot's moving joints
        #test self.robot-object collisions
        for o in xrange(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index,o)):
                return False;
        #test self.robot-terrain collisions
        for o in xrange(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index,o)):
                return False;
        #test self.robot self-collisions
        if any(collider.robotSelfCollisions(self.robot.index)):
            return False
        return True
