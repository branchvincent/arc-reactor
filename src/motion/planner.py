#!/usr/bin/python

import sys
from klampt import *
from klampt.vis.glrobotprogram import *
from klampt.model import ik,coordinates,config,cartesian_trajectory,trajectory
from klampt.model.collide import WorldCollider
from klampt.plan.cspace import CSpace,MotionPlan
from klampt.math import so3,se3
import random
import importlib
import os
import time
import json
import copy
ee_local=[-0.015,-0.02,0.28]
# ee_local=[0,0,0]
box_release_offset=[0,0,0.06]
idle_position=[0.6,0,1]
shelf_position=[1.0,0.4,0.235]
# approach_p1=[0.6,0.2+shelf_position[1],1]
# approach_p2=[0.6,-0.1+shelf_position[1],1]
order_box_min=[0.36,0.65,0.5]
order_box_max=[0.5278,0.904,0.5]
angle_to_degree=57.296
ee_link=6
control_rate=30 #controlling the robot with 20 Hz




def pick_up(world,item,target_box):
	"""
	This function will return a motion plan that will pick up the target item from the shelf and drop it to the tote.
	Inputs:
		- world model: including klampt models for the robot, the shelf and the target container
		- item: position/orientation of the target item, ideal surfaces and approach directions for vacuum or preferred grasp configurations for known item
			-- position: item position
			-- vacuum_offset: hacked parameter for each item, added to the high of the item to find the grasp position for the vacuum
			-- drop offset: hacked parameter for each item, added to the high of the order box bottom high to find the drop position for the vacuum
		- target_box: the ID of the target order box, a default droping position for that order box. 
			-- drop position: right now is above the center of the box
			-- position: box position
	outputs:
		a list of milestones=(t, {
              'robot': q,
              'gripper': [0,0,0],
              'vaccum': [0]
            })
        - t: planned time to reach this configuration
        - robot q: raw output from the klampt simulation. q=[q0,q1,q2,q3,q4,q5,q6]. q0 is alwasy 0. The signs for q3 and q5 is flipped on the real robot. 
        Also all the values are in radius, while the real robot joint values are in degree
        - gripper control
        - vaccum: 0-off 1-on
	"""
	
	#init
	robot=world.robot(0)
	motion_milestones=[]
	current_config=robot.getConfig()
	curr_position=robot.link(ee_link).getWorldPosition(ee_local)
	curr_orientation,p=robot.link(ee_link).getTransform()
	current_T=[curr_orientation,curr_position]
	# item_position=item['position']
	# print 'item_position',item_position
	# print vectorops.div(vectorops.add(item['bbox'][0],item['bbox'][1]),2.0)
	# time.sleep(1)
	item_position=vectorops.div(vectorops.add(item['bbox'][0],item['bbox'][1]),2.0)
	item_vacuum_offset=item['vacuum_offset']
	drop_offset=item['drop offset']
	drop_position=target_box['drop position']
	box_bottom_high=target_box['position'][2]
	vaccum_approach_distance=[0,0,0.03]
	#setting some constant parameters and limits
	
	max_end_effector_v=0.8 # 0.8m/s
	test_cspace=TestCSpace(Globals(world))
	approach_p1=[0.6,0.2+shelf_position[1],1]
	approach_p2=[0.6,-0.2+shelf_position[1],1]



	#list of T and time

	#from current position to the start position, in 2 seconds
	if item_position[1]*shelf_position[0]>item_position[0]*shelf_position[1]:
		start_position=approach_p1
	else:
		start_position=approach_p2
	start_position[2]=item_position[2]+item_vacuum_offset[2]
	d=vectorops.distance(start_position[:1],item_position[:1])
	dx=item_position[0]-start_position[0]
	dy=item_position[1]-start_position[1]
	end_T=[[0,0,-1, -dy/d,dx/d,0, dx/d,dy/d,0],start_position]
	l=vectorops.distance(current_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,2,control_rate,current_T,end_T,0,0,1)
	if not motion_milestones:
		return False
	#from start position to the pregrasp position
	start_T=copy.deepcopy(end_T)
	end_T=[[0,0,-1, 0,1,0,1,0,0],vectorops.add(item_position,vectorops.add(item_vacuum_offset,vaccum_approach_distance))]
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,0,0,1)
	if not motion_milestones:
		return False
	#start the vacuum
	motion_milestones.append(make_milestone(2,robot.getConfig(),1,0))
	# lower the vacuum
	temp=start_T
	start_T=copy.deepcopy(end_T)
	temp[1]=vectorops.add(item_position,item_vacuum_offset)
	end_T=temp
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,0,1)
	if not motion_milestones:
		return False



	#pick the item
	temp=start_T
	start_T=copy.deepcopy(end_T)
	temp[1]=vectorops.add(vectorops.add(item_position,item_vacuum_offset),vaccum_approach_distance)
	end_T=temp
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		return False
	#retract
	temp=start_T
	start_T=copy.deepcopy(end_T)
	temp[1]=start_position
	end_T=temp
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		return False
	#go to tote
	temp=start_T
	start_T=copy.deepcopy(end_T)
	temp[1]=drop_position
	end_T=temp
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,0)
	# print 'go to tote'
	if not motion_milestones:
		return False

	#drop item
	start_T=copy.deepcopy(end_T)
	temp=end_T

	temp[1][2]=box_bottom_high+item['drop offset']
	end_T=temp
	end_T[1]=[0.25,0.9,0.4]
	l=vectorops.distance(start_T[1],end_T[1])
	# print start_T[1]
	# print end_T[1]
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,0)
	if not motion_milestones:
		return False
	#turn off vacuum
	motion_milestones.append(make_milestone(2,robot.getConfig(),0,0))
	# print 'drop item'

	# f=open('test.json','w')
	# json.dump(motion_milestones,f)
	# f.close()
	return motion_milestones



def stow(world,item,target_box):
	"""
	This function will return a motion plan that will pick up the target item from the tote and place it to the shelf.
	Inputs:
		- world model: including klampt models for the robot, the shelf and the target container
		- item: position/orientation of the target item, ideal surfaces and approach directions for vacuum or preferred grasp configurations for known item
			-- position: item position
			-- vacuum_offset: hacked parameter for each item, added to the high of the item to find the grasp position for the vacuum
			-- drop offset: hacked parameter for each item, added to the high of the order box bottom high to find the drop position for the vacuum
		- target_box: hacked from pick function. right now drop position is a goal position for the target item on the shelf 
	outputs:
		a list of milestones=(t, {
              'robot': q,
              'gripper': [0,0,0],
              'vaccum': [0]
            })
        - t: planned time to reach this configuration
        - robot q: raw output from the klampt simulation. q=[q0,q1,q2,q3,q4,q5,q6]. q0 is alwasy 0. The signs for q3 and q5 is flipped on the real robot. 
        Also all the values are in radius, while the real robot joint values are in degree
        - gripper control
        - vaccum: 0-off 1-on
	"""
	robot=world.robot(0)
	motion_milestones=[]
	current_config=robot.getConfig()
	curr_position=robot.link(ee_link).getWorldPosition(ee_local)
	curr_orientation=robot.link(ee_link).getTransform()[0]
	current_T=[curr_orientation,curr_position]
	# item_position=item['position']
	item_position=vectorops.div(vectorops.add(item['bbox'][0],item['bbox'][1]),2.0)
	item_vacuum_offset=item['vacuum_offset']
	drop_offset=item['drop offset']
	drop_position=target_box['drop position']
	box_bottom_high=target_box['position'][2]
	vaccum_approach_distance=[0,0,0.15]
	#setting some constant parameters and limits
	max_end_effector_v=0.7 # 0.8m/s
	approach_p1=[0.6,0.2+shelf_position[1],1]
	approach_p2=[0.6,-0.2+shelf_position[1],1]
	test_cspace=TestCSpace(Globals(world))
	vectorops
	#list of T and time

	#from current position to the start position, in 2 seconds
	# if item_position[1]*shelf_position[0]>item_position[0]*shelf_position[1]:
	# 	start_position=approach_p1
	# else:
	# 	start_position=approach_p2
	# start_position[2]=item_position[2]+item_vacuum_offset[2]
	# d=vectorops.distance(start_position[:1],item_position[:1])
	# dx=item_position[0]-start_position[0]
	# dy=item_position[1]-start_position[1]
	# end_T=[[0,0,-1, -dy/d,dx/d,0, dx/d,dy/d,0],start_position]
	# l=vectorops.distance(current_T[1],end_T[1])
	# motion_milestones=add_milestones(robot,motion_milestones,l/max_end_effector_v,control_rate,current_T,end_T,0,0,1)

	#from start position to the pregrasp position
	start_T=copy.deepcopy(current_T)
	end_T=[[0,0,-1, 0,1,0,1,0,0],vectorops.add(item_position,vectorops.add(item_vacuum_offset,vaccum_approach_distance))]
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,2,control_rate,start_T,end_T,0,0,0)
	if not motion_milestones:
		return False

	#start the vacuum
	motion_milestones.append(make_milestone(2,robot.getConfig(),1,0))
	# lower the vacuum
	temp=start_T
	start_T=copy.deepcopy(end_T)
	temp[1]=vectorops.add(item_position,item_vacuum_offset)
	end_T=temp
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,0,0)
	if not motion_milestones:
		return False

	#pick the item
	temp=start_T
	start_T=copy.deepcopy(end_T)
	temp[1]=vectorops.add(vectorops.add(item_position,item_vacuum_offset),vaccum_approach_distance)
	end_T=temp
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,0)
	if not motion_milestones:
		return False
	#retract
	temp=start_T
	curr_position=robot.link(ee_link).getWorldPosition(ee_local)
	curr_orientation=robot.link(ee_link).getTransform()[0]
	start_T=[curr_orientation,curr_position]
	if drop_position[1]*shelf_position[0]>drop_position[0]*shelf_position[1]:
		start_position=approach_p1
	else:
		# print 'p2!!'
		start_position=approach_p2
	start_position[2]=drop_position[2]+item_vacuum_offset[2]
	d=vectorops.distance(start_position[:1],drop_position[:1])
	dx=drop_position[0]-start_position[0]
	dy=drop_position[1]-start_position[1]
	end_T=[[0,0,-1, -dy/d,dx/d,0, dx/d,dy/d,0],start_position]
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	# print 'retract'
	if not motion_milestones:
		return False

	#go to shelf
	curr_position=robot.link(ee_link).getWorldPosition(ee_local)
	curr_orientation=robot.link(ee_link).getTransform()[0]
	start_T=[curr_orientation,curr_position]
	temp=[curr_orientation,drop_position]
	temp[1][2]+=drop_offset
	end_T=temp
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		return False
	# print 'go to shelf'
	#drop item
	temp=start_T
	start_T=copy.deepcopy(end_T)
	end_T=temp
	end_T[1][2]=box_bottom_high+item['drop offset']
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,0,0,0)
	if not motion_milestones:
		return False
	#turn off vacuum
	motion_milestones.append(make_milestone(2,robot.getConfig(),0,0))
	
	#come back
	temp=start_T
	start_T=copy.deepcopy(end_T)
	end_T=temp
	end_T[1]=curr_position
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,0,0,0)
	if not motion_milestones:
		return False
	
	# f=open('test.json','w')
	# json.dump(motion_milestones,f)
	# f.close()
	return motion_milestones



def interpolate(start_T,end_T,u,flag):
    R1,t1=start_T
    R2,t2=end_T
    T=se3.interpolate(start_T,end_T,u)
    R,t=T
    if flag:
        ik_local=[ee_local,vectorops.add(ee_local,[0.01,0,0]),vectorops.add(ee_local,[0,0,0.01])]
        ik_world=[t,vectorops.add(t,[R[0]/100.0,R[1]/100.0,R[2]/100.0]),vectorops.add(t,[R[6]/100.0,R[7]/100.0,R[8]/100.0])]
    else:
        ik_local=[ee_local,vectorops.add(ee_local,[0.01,0,0])]
        ik_world=[t,vectorops.add(t,[R[0]/100.0,R[1]/100.0,R[2]/100.0])]
    return [ik_local,ik_world]

def make_milestone(t,q,vacuum_status,simulation_status):
	milestone=(t, {
              'robot': q,
              'gripper': [0,0,0],
              'vacuum': [vacuum_status], 
              'simulation': simulation_status
            })
	return milestone


def add_milestones(test_cspace,robot,milestones,t,control_rate,start_T,end_T,vacuum_status,simulation_status,ik_indicator):
	# q=robot.getConfig()
	# milestones.append(make_milestone(0.5,q,vacuum_status))
	if t<0.3:
		t=0.3
	steps=t*control_rate
	t_step=1.0/control_rate
	# print "start config",robot.getConfig()
	i=0
	while i<=steps:
		q_old=robot.getConfig()
		u=i*1.0/steps
		# print u
		[local_p,world_p]=interpolate(start_T,end_T,u,ik_indicator)
		goal = ik.objective(robot.link(ee_link),local=local_p,world=world_p)
		s=ik.solve_global(goal)
		# s=ik.solve_nearby(goal,maxDeviation=5,feasibilityCheck=test_function)
		q=robot.getConfig()
		n=0
		# print i
		# print s
		# print test_cspace.feasible(q)
		while n<10:
			if s and test_cspace.feasible(q):
				break
			else:
				# print "no feasible ik solution found"
				# print world_p
				robot.setConfig(q_old)
				# s=ik.solve_nearby(goal,maxDeviation=5,feasibilityCheck=test_function)
				s=ik.solve_global(goal)
	        	q=robot.getConfig()
	        	# print 's',s
	        	# print 'feasible test:',test_cspace.feasible(q)
	        	n+=1
		if s and test_cspace.feasible(q):
			milestones.append(make_milestone(t_step,q,vacuum_status,simulation_status))
			i+=1
		else:
			print 'no feasible solution can be found!!!!!'
			return False
	return milestones


class Globals:
    def __init__(self,world):
        self.world = world
        self.robot = world.robot(0)
        self.collider = WorldCollider(world)


class TestCSpace(CSpace):
    """A CSpace defining the feasibility constraints of the robot"""
    def __init__(self,globals):
        CSpace.__init__(self)
        self.globals = globals
        self.robot = globals.robot
        #initial whole-body configuratoin
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
        #TODO: this could be much more efficient if you only tested the robot's moving joints
        #test robot-object collisions
        for o in xrange(world.numRigidObjects()):
            if any(collider.robotObjectCollisions(self.robot.index,o)):
                return False;
        #test robot-terrain collisions
        for o in xrange(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index,o)):
                return False;
        #test robot self-collisions
        if any(collider.robotSelfCollisions(self.robot.index)):
            return False
        return True

def test_function():
    return True


