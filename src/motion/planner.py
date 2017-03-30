#!/usr/bin/python

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
ee_local=[0.0,0.0,0.37]
# ee_local=[0,0,0]
box_release_offset=[0,0,0.06]
# approach_p1=[0.6,0.2+shelf_position[1],1]
# approach_p2=[0.6,-0.1+shelf_position[1],1]
order_box_min=[0.36,0.65,0.5]
order_box_max=[0.5278,0.904,0.5]
angle_to_degree=57.296
ee_link=6
control_rate=20 #controlling the robot with 20 Hz
max_end_effector_v=0.3 # max end effector move speed
max_change=200.0/180.0*3.14159 #the max change between raw milestones is 20 degree
milestone_check_max_change=5.0/180.0*3.14159 #the max change between milestones is 5 degree
milestone_check_max_speed=60/180.0*3.14159


def joint_space_rotate(motion_milestones,current_p,target_p,robot,vacuum_status):
	theta1=math.atan2(current_p[1],current_p[0])
	theta2=math.atan2(target_p[1],target_p[0])
	delta=theta2-theta1
	if delta>math.pi:
		delta=delta-2*math.pi
	elif delta<-math.pi:
		delta=delta+2*math.pi
	if delta>0:
		#ccw rotate
		while delta>0:
			q=robot.getConfig()
			q[1]+=0.05
			q[6]+=0.05
			motion_milestones.append(make_milestone(0.05,q,vacuum_status,vacuum_status))
			robot.setConfig(q)
			delta-=0.05
	else:
		while delta<0:
			q=robot.getConfig()
			q[1]-=0.05
			q[6]-=0.05
			motion_milestones.append(make_milestone(0.05,q,vacuum_status,vacuum_status))
			robot.setConfig(q)
			delta+=0.05
	return motion_milestones


def pick_up(world,item,target_box,target_index):
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
			-- box_limit:box_min and box_max, the bounding box for the target container
		- target_index: the index of the target item in the world, this is used for feasible placement finding
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
		check_points for the motion plan:
			-- a list of transform matrixs for the end-effector at key points for the plan
	"""

	#init
	robot=world.robot(0)

	current_config=robot.getConfig()
	curr_position=robot.link(ee_link).getWorldPosition(ee_local)
	curr_orientation,p=robot.link(ee_link).getTransform()
	current_T=[curr_orientation,curr_position]
	item_position=vectorops.div(vectorops.add(item['bbox'][0],item['bbox'][1]),2.0)
	item_vacuum_offset=item['vacuum_offset']
	drop_offset=item['drop offset']
	drop_position=target_box['drop position']
	# print drop_position
	# if target_box['drop position']:
	#   drop_position=target_box['drop position']
	# else:
	#   drop_position=find_drop_position(item['bbox'],order_box_min,order_box_max,world)
	box_bottom_high=target_box['position'][2]
	vaccum_approach_distance=[0,0,0.03]
	#setting outputs
	check_points=[]
	motion_milestones=[]
	test_cspace=TestCSpace(Globals(world))

	#find a placement position if no goal position is given by the input
	if target_box['drop position']:
		drop_position=target_box['drop position']
		print drop_position
		box_bottom_high=target_box['position'][2]
	else:
		print "here!!!!!!!!"
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
			print goal_T[1]
			# goal_T[1]=[1.10298067096586, 0.3038671358694375, 0.791611841275841]
			if check_placement(world,goal_T,target_index):
				flag=0
				drop_position=goal_T[1]
				box_bottom_high=goal_T[1][2]
			n+=1
		if flag:
			print "can't find a feasible placement"
			world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
			return False
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
				print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		print 'find a placement:',goal_T[1]
		world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])


	check_points.append(current_T)
	motion_milestones=joint_space_rotate(motion_milestones,p,item_position,robot,0)
	curr_position=robot.link(ee_link).getWorldPosition(ee_local)
	curr_orientation,p=robot.link(ee_link).getTransform()
	current_T=[curr_orientation,curr_position]
	check_points.append(current_T)
	#move the robot from current position to a start position that is above the target item
	start_position=vectorops.add(item_position,[0,0,0.4])
	start_position[2]=min(0.4,start_position[2])
	end_T=[[1,0,0,0,-1,0,0,0,-1],start_position]
	check_points.append(end_T)
	l=vectorops.distance(current_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,current_T,end_T,0,0,1)
	if not motion_milestones:
		print "can't find a feasible path to start position"
		return False

	#start the vacuum
	motion_milestones.append(make_milestone(1,robot.getConfig(),1,0))

	#lower the vacuum
	start_T=end_T
	end_T[1]=vectorops.add(item_position,item_vacuum_offset)
	check_points.append(end_T)
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		print "can't find a feasible path to lower the vacuum"
		return False

	#pick up the item
	start_T=end_T
	end_T[1][2]+=0.4
	l=vectorops.distance(start_T[1],end_T[1])
	check_points.append(end_T)
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		print "can't find a feasible path to pick up the item"
		return False

	curr_orientation,p=robot.link(ee_link).getTransform()
	motion_milestones=joint_space_rotate(motion_milestones,p,drop_position,robot,1)
	# while drop_position[0]*p[1]<p[0]*drop_position[1]:
	# 	q=robot.getConfig()
	# 	q[1]+=0.05
	# 	q[6]+=0.05
	# 	motion_milestones.append(make_milestone(0.05,q,1,1))
	# 	robot.setConfig(q)
	# 	curr_orientation,p=robot.link(ee_link).getTransform()



	#move the item to the start position for droping
	start_T=end_T
	end_T[1][0]=drop_position[0]
	end_T[1][1]=drop_position[1]
	check_points.append(end_T)
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		print "can't find a feasible path to the drop position"
		return False

	#lower the item
	start_T=end_T
	end_T[1]=vectorops.add(drop_position,drop_offset)
	check_points.append(end_T)
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		print "can't find a feasible path to lower the item"
		return False

	#turn off the vacuum
	motion_milestones.append(make_milestone(1,robot.getConfig(),0,0))

	#raise the robot
	start_T=end_T
	end_T[1][2]=0.45
	check_points.append(end_T)
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,0,0,0)
	if not motion_milestones:
		print "can't find a feasible path to raise the robot"
		return False

	fix_milestones(motion_milestones)
	return motion_milestones

def check_placement(world,T,target_index):
	world.rigidObject(target_index).setTransform(T[0],T[1])
	glist_target=[]
	glist_target.append(world.rigidObject(target_index).geometry())
	glist_terrain=[]
	glist_object=[]
	for i in xrange(world.numTerrains()):
		t = world.terrain(i)
		g = t.geometry()
		if g != None and g.type()!="":
			glist_terrain.append(g)
	for i in xrange(world.numRigidObjects()):
		o = world.rigidObject(i)
		g = o.geometry()
		if g != None and g.type()!="" and i!=target_index:
			glist_object.append(g)
	if any(collide.group_collision_iter(glist_target,glist_object)):
	# if any(collide.self_collision_iter(glist_target+glist_object)):
		print 'objects colliding!'
		return False
	if any(collide.group_collision_iter(glist_target,glist_terrain)):
	# if any(collide.self_collision_iter(glist_target+glist_terrain)):
		print 'terrain colliding!'
		return False
	return True

def stow(world,item,target_box,target_index):
	"""
	This function will return a motion plan that will pick up the target item from the tote and place it to the shelf.
	Inputs:
		- world model: including klampt models for the robot, the shelf and the target container
		- item: position/orientation of the target item, ideal surfaces and approach directions for vacuum or preferred grasp configurations for known item
			-- position: item position
			-- vacuum_offset: hacked parameter for each item, added to the high of the item to find the grasp position for the vacuum
			-- drop offset: hacked parameter for each item, added to the high of the order box bottom high to find the drop position for the vacuum
		- target_box: hacked from pick function. right now drop position is a goal position for the target item on the shelf
			--box_limit:box_min and box_max, the bounding box for the target container
		- target_index: the index of the target item in the world, this is used for feasible placement finding
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
	curr_orientation,p=robot.link(ee_link).getTransform()
	current_T=[curr_orientation,curr_position]
	# item_position=item['position']
	item_position=vectorops.div(vectorops.add(item['bbox'][0],item['bbox'][1]),2.0)
	item_vacuum_offset=item['vacuum_offset']
	drop_offset=item['drop offset']

	vaccum_approach_distance=[0,0,0.15]

	test_cspace=TestCSpace(Globals(world))




	#find a stowing position if no goal position is given by the input
	if target_box['drop position']:
		drop_position=target_box['drop position']
		print drop_position
		box_bottom_high=target_box['position'][2]
	else:
		print "here!!!!!!!!"
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
			print goal_T[1]
			# goal_T[1]=[1.10298067096586, 0.3038671358694375, 0.791611841275841]
			if check_placement(world,goal_T,target_index):
				flag=0
				drop_position=goal_T[1]
				box_bottom_high=goal_T[1][2]
			n+=1
		if flag:
			print "can't find a feasible placement"
			world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
			return False
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
				print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		print 'find a placement:',goal_T[1]
		world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])




	motion_milestones=joint_space_rotate(motion_milestones,p,item_position,robot,0)
	curr_position=robot.link(ee_link).getWorldPosition(ee_local)
	curr_orientation,p=robot.link(ee_link).getTransform()
	current_T=[curr_orientation,curr_position]
	#move the robot from current position to a start position that is above the target item
	start_position=vectorops.add(item_position,[0,0,0.3])
	end_T=[[1,0,0,0,-1,0,0,0,-1],start_position]
	l=vectorops.distance(current_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,current_T,end_T,0,0,1)
	if not motion_milestones:
		print "can't find a feasible path to start position"
		return False

	#start the vacuum
	motion_milestones.append(make_milestone(1,robot.getConfig(),1,0))

	#lower the vacuum
	start_T=end_T
	end_T[1]=vectorops.add(item_position,item_vacuum_offset)
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		print "can't find a feasible path to lower the vacuum"
		return False

	#pick up the item
	start_T=end_T
	end_T[1][2]+=0.4
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		print "can't find a feasible path to pick up the item"
		return False


	curr_orientation,p=robot.link(ee_link).getTransform()
	motion_milestones=joint_space_rotate(motion_milestones,p,drop_position,robot,1)
	# while drop_position[0]*p[1]>p[0]*drop_position[1]:
	# 	q=robot.getConfig()
	# 	q[1]-=0.05
	# 	q[6]-=0.05
	# 	motion_milestones.append(make_milestone(0.05,q,1,1))
	# 	robot.setConfig(q)
	# 	curr_orientation,p=robot.link(ee_link).getTransform()



	#move the item to the start position for droping
	start_T=end_T
	end_T[1][0]=drop_position[0]
	end_T[1][1]=drop_position[1]
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		print "can't find a feasible path to drop position"
		return False

	#lower the item
	start_T=end_T
	end_T[1]=vectorops.add(drop_position,drop_offset)
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,1,1,1)
	if not motion_milestones:
		print "can't find a feasible path to lower the item"
		return False

	#turn off the vacuum
	motion_milestones.append(make_milestone(1,robot.getConfig(),0,0))

	#raise the robot
	start_T=end_T
	end_T[1][2]=0.45
	l=vectorops.distance(start_T[1],end_T[1])
	motion_milestones=add_milestones(test_cspace,robot,motion_milestones,l/max_end_effector_v,control_rate,start_T,end_T,0,0,0)
	if not motion_milestones:
		print "can't find a feasible path to raise the robot"
		return False


	fix_milestones(motion_milestones)
	return motion_milestones

def fix_milestones(motion_milestones):
	old_config=motion_milestones[0][1]['robot']
	i=1
	while i<len(motion_milestones):
		new_config=motion_milestones[i][1]['robot']
		d_config=max(max(vectorops.sub(new_config,old_config)),-min(vectorops.sub(new_config,old_config)))
		speed_config=d_config/motion_milestones[i][0]
		if d_config>milestone_check_max_change:#the max change between milestones is 5 degree:
			# print 'd_config',d_config
			new_milestone=make_milestone(motion_milestones[i][0],vectorops.div(vectorops.add(motion_milestones[i-1][1]['robot'],motion_milestones[i][1]['robot']),2),motion_milestones[i-1][1]['vacuum'][0],motion_milestones[i-1][1]['simulation'])
			motion_milestones.insert(i,new_milestone)
			continue
		elif speed_config>milestone_check_max_speed:
			new_milestone=make_milestone(d_config/(milestone_check_max_speed-0.1),motion_milestones[i][1]['robot'],motion_milestones[i][1]['vacuum'][0],motion_milestones[i][1]['simulation'])
			motion_milestones[i]=new_milestone
			i+=1
			old_config=new_config
		else:
			i+=1
			old_config=new_config
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
		ik_local=[ee_local,vectorops.add(ee_local,[0,0,0.01])]
		ik_world=[t,vectorops.add(t,[R[6]/100.0,R[7]/100.0,R[8]/100.0])]
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
	if t<0.3:
		t=0.3
	steps=t*control_rate
	t_step=1.0/control_rate
	# print "start config",robot.getConfig()
	i=0
	start_q=robot.getConfig()[3]
	while i<=steps:
		q_old=robot.getConfig()
		u=i*1.0/steps
		# print u
		[local_p,world_p]=interpolate(start_T,end_T,u,ik_indicator)
		goal = ik.objective(robot.link(ee_link),local=local_p,world=world_p)
		s=ik.solve_global(goal)
		# s=ik.solve_nearby(goal,maxDeviation=1000,feasibilityCheck=test_function)
		q=robot.getConfig()
		n=0
		# print i
		# print s
		# print test_cspace.feasible(q)

		flag = 1
		if (max(vectorops.sub(q_old,q))>max_change) or (min(vectorops.sub(q_old,q))<(-max_change)) or q[3]*start_q<0:
			print "too much change!"
			print vectorops.sub(q_old,q)
			flag=0

		while n<30:
			if flag and (s and test_cspace.feasible(q)) :
				break
			else:
				# print "no feasible ik solution found"
				# print world_p
				robot.setConfig(q_old)
				s=ik.solve_global(goal)
				# s=ik.solve_nearby(goal,maxDeviation=1000,feasibilityCheck=test_function)
				q=robot.getConfig()
				if (max(vectorops.sub(q_old,q))>max_change) or (min(vectorops.sub(q_old,q))<(-max_change)) or q[3]*start_q<0:
					print "too much change!"
					flag=0
				else:
					flag=1
				# print 's',s
				# print 'feasible test:',test_cspace.feasible(q)
				n+=1
		if flag and s and test_cspace.feasible(q):
			m_change=max(max(vectorops.sub(q_old,q)),-min(vectorops.sub(q_old,q)))
			milestones.append(make_milestone(t_step,q,vacuum_status,simulation_status))
			i+=1
		else:
			print 'no feasible solution can be found!!!!!'
			print s
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
