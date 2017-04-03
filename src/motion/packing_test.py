#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.vis.glrobotprogram import *
from klampt.model import ik,coordinates,config,cartesian_trajectory,trajectory
from klampt.model.collide import WorldCollider
from klampt.plan import cspace
from klampt.math import so3,se3
import random
import importlib
import os
import time
import json
import planner
import math

box_limit={}
box_limit['1A5']=[[-0.1715,-0.1395,0.05],[0.1715,0.1395,0.121]]


#load related files for objects 
object_template_fn = '../../data/objects/object_template.obj'
objects = {}
objects['apc2017'] = [f for f in sorted(os.listdir('../../data/objects/apc2017'))]
objects['apc2017'].remove("gen_thumbnail")
objects['apc2017'].remove("convert_psd")

object_geom_file_patterns = {
	'apc2017':['../../data/objects/apc2017/%s/mesh.ply']
}
#default mass for objects whose masses are not specified, in kg
default_object_mass = 0.5
object_masses = {
	'apc2017':dict(),
}


def generate_orders(n,box,box_fit_item,box_volume,item_dictionary,box_name,utilization_rate=0):
	# utilization_rate is the total item volume/box volume. range is (0,1). Higher value means the items are taking much space of the box
		if not box:
			if n==2:
				box=random.randint(0,1)
			elif n==3:
				box=random.randint(2,3)
			else:
				box=4
		fitting_item=box_fit_item[box]
		items=random.sample(xrange(0,len(fitting_item)-1),n)
		box_v=box_volume[box]
		item_v=0
		for item in items:
			item_v=item_v+item_dictionary[fitting_item[item]]
		count=0
		flag=0
		while item_v>box_v or item_v/box_v<utilization_rate:
			items=random.sample(xrange(0,len(fitting_item)-1),n)
			box_v=box_volume[box]
			item_v=0
			for item in items:
				item_v=item_v+item_dictionary[fitting_item[item]]
			count+=1
			if count>50:
				flag=1
				break
		if flag:
			print ("timeout! can't reach the utilization rate with %d items",n)
		else:
			print "size_id:",box_name[box]
			print "item volume/box volume:",item_v/box_v*100
			order_list=[]
			for item in items:
				order_list.append(fitting_item[item])
			return order_list


def make_object(object_set,objectname,world):
	"""Adds an object to the world using its geometry / mass properties
	and places it in a default location (x,y)=(0,0) and resting on plane."""
	for pattern in object_geom_file_patterns[object_set]:
		objfile = pattern%(objectname,)
		objmass = object_masses[object_set].get('mass',default_object_mass)
		f = open(object_template_fn,'r')
		pattern = ''.join(f.readlines())
		f.close()
		f2 = open("temp.obj",'w')
		f2.write(pattern % (objfile,objmass))
		f2.close()
		nobjs = world.numRigidObjects()
		if world.loadElement('temp.obj') < 0 :
			continue
		assert nobjs < world.numRigidObjects(),"Hmm... the object didn't load, but loadElement didn't return -1?"
		obj = world.rigidObject(world.numRigidObjects()-1)
		obj.setTransform(*se3.identity())
		bmin,bmax = obj.geometry().getBB()
		T = obj.getTransform()
		spacing = 0.006
		T = (T[0],vectorops.add(T[1],(-(bmin[0]+bmax[0])*0.5,-(bmin[1]+bmax[1])*0.5,-bmin[2]+spacing)))
		obj.setTransform(*T)
		obj.appearance().setColor(0.2,0.5,0.7,1.0)
		obj.setName(objectname)
		return obj
	raise RuntimeError("Unable to load object name %s from set %s"%(objectname,object_set))

def check_placement(world,T,target_index,box_limit):
	world.rigidObject(target_index).setTransform(T[0],T[1])
	bb1,bb2=world.rigidObject(target_index).geometry().getBB()
	x=(bb1[0]+bb2[0])/2.0
	y=(bb1[1]+bb2[1])/2.0
	z=(bb1[2]+bb2[2])/2.0
	if x<box_limit[0][0] or x>box_limit[1][0] or y<box_limit[0][1] or y>box_limit[1][1] or z<box_limit[0][2]:
		return False
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
		return False
	# if any(collide.self_collision_iter(glist_target+glist_object)):
		# print 'objects colliding!'
		
	if any(collide.group_collision_iter(glist_target,glist_terrain)):
		return False
	# if any(collide.self_collision_iter(glist_target+glist_terrain)):
		# print 'terrain colliding!'
		
	

	return True

class MyGLViewer(GLSimulationPlugin):
	def __init__(self,world):
		GLSimulationPlugin.__init__(self,world)
		self.world = world
		self.target=0
		self.reset_T=[]
		self.start_time=0
		self.packing_mode=0
		self.box_limit=box_limit['1A5']
		self.score=[]
		# print self.world.terrain("order_box").geometry().getBB()
		for i in range(world.numRigidObjects()):
			self.sim.body(self.sim.world.rigidObject(i)).enable(False)
			self.reset_T.append(self.world.rigidObject(i).getTransform())
			# print self.world.rigidObject(i).getTransform()[1]
		
	def control_loop(self):
		if self.packing_mode==0 and self.sim.getTime()>self.start_time+1+2*self.target:
			if self.target<self.world.numRigidObjects():
				print "pacing the object!"
				obj=self.sim.body(self.world.rigidObject(self.target))
				obj.setObjectTransform(so3.identity(),[0,0,self.box_limit[1][2]+0.15])
				print "set to:",[0,0,self.box_limit[1][2]+0.15]
				print "is :",self.world.rigidObject(self.target).getTransform()[1]
				obj.enable(True)
				self.target+=1
			else:
				self.make_score()
				print self.score
				self.reset_items()
				self.packing_mode=1
		elif self.packing_mode==1 and self.sim.getTime()>self.start_time+1+2*self.target:
			if self.target<self.world.numRigidObjects():
				print "pacing the object!"
				obj=self.sim.body(self.world.rigidObject(self.target))
				good_t=[0,0,self.box_limit[1][2]+0.4]
				for i in range(30):
					R,t=self.random_feasible_packing()
					if t[2]<good_t[2]:
						print "better position!"
						good_t=t
				# self.world.rigidObject(self.target).setTransform(R,good_t)
				print good_t
				good_t[2]=max(0.2,good_t[2])
				print check_placement(self.world,[R,good_t],self.target,self.box_limit)
				obj.setObjectTransform(R,good_t)
				obj.enable(True)
				self.target+=1
			else:
				self.make_score()
				print self.score
				# self.reset_items()
				self.packing_mode=2
		elif self.packing_mode==2:
			print self.score
			self.packing_mode=3

	def reset_items(self):
		for i in range(world.numRigidObjects()):	
			R,t=self.reset_T[i]
			obj=self.sim.body(self.world.rigidObject(i))
			obj.setObjectTransform(R,t)
			obj.enable(False)
			self.target=0
			self.start_time=self.sim.getTime()


	def make_score(self):
		score=0
		highest=0
		for i in range(self.world.numRigidObjects()):
			bb1,bb2= self.world.rigidObject(i).geometry().getBB()
			t=vectorops.div(vectorops.add(bb1,bb2),2.0)
			h=self.world.rigidObject(i).geometry().getBB()[1][2]
			print h
			if t[0]>self.box_limit[0][0] and t[0]<self.box_limit[1][0] and t[1]>self.box_limit[0][1] and t[1]<self.box_limit[1][1]:
				score+=10
			if h>highest:
				highest=h
		if score==10*self.world.numRigidObjects():
			if highest<self.box_limit[1][2]:
				score+=10
		self.score.append(score)

	def random_feasible_packing(self):
		box_min,box_max=box_limit['1A5']
		target_index=self.target
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
			if check_placement(world,goal_T,target_index,self.box_limit):
				flag=0
				drop_position=goal_T[1]
			n+=1
		if flag:
			# print "can't find a feasible placement"
			world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
			return [origin_T[0],[0,0,self.box_limit[1][2]+0.15]]
		n=0
		check_flag=1
		low_bound=0
		high_bound=z
		while n<5:
			new_z=0.5*(low_bound+high_bound)
			goal_T[1]=[x,y,new_z]
			if check_placement(world,goal_T,target_index,self.box_limit):
				drop_position=goal_T[1]
				high_bound=new_z
				# print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		# print 'find a placement:',goal_T[1]
		# print '************'
		# print check_placement(world,[origin_T[0],drop_position],0)
		# print "---------------"
		world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
		return [origin_T[0],drop_position]

	def mousefunc(self,button,state,x,y):
		#Put your mouse handler here
		#the current example prints out the list of objects clicked whenever
		#you right click
		if button==2:
			if state==0:
				print [o.getName() for o in self.click_world(x,y)]
				return
		GLPluginInterface.mousefunc(self,button,state,x,y)

	def print_help(self):
		GLSimulationPlugin.print_help(self)
		print 'Drive keys:',sorted(self.keymap.keys())

	def keyboardfunc(self,c,x,y):
		if c=='a':
			if self.target<self.world.numRigidObjects():
				print "moving the object!"
				obj=self.sim.body(self.world.rigidObject(self.target))
				obj.setTransform(so3.identity(),[0,0,0.4])
				obj.enable(True)
				self.target+=1
			
			# self.world.remove(self.world.rigidObject(self.world.numRigidObjects()-1))
		GLSimulationPlugin.keyboardfunc(self,c,x,y)
		self.refresh()



if __name__ == "__main__":
	# if len(sys.argv)<=1:
	# 	print "USAGE: test_planner.py ../../data/ARC_packing.xml"
	# 	exit()
	world = WorldModel()
	res = world.readFile("../../data/ARC_packing.xml")
	if not res:
		raise RuntimeError("Unable to load model "+fn)
	
	with open("pick_task_generator.json") as json_file:
			data=json.load(json_file)
	box_fit_item=data['box_fit_item']
	box_volume=data['box_volume']
	item_dictionary=data['item_dictionary']
	box_name=data['box_name']	

	shelved=[]
	order_box=2
	n=3
	order_list=generate_orders(n,order_box,box_fit_item,box_volume,item_dictionary,box_name,0.5)
	name_index={}
	for i in range(40):
		name_index[box_fit_item[4][i]]=i
	# print order_list
	for i in range(n):
			dataset=random.choice(objects.keys())
			# index = random.randint(0,len(objects[dataset])-1)
			index=name_index[order_list[i]]
			# index=3
			objname = objects[dataset][index]
			# print dataset,objname
			shelved.append((dataset,objname))

	# test=[]
	# test.append([-0.0468572843527858+1, 0.01388161073849592, 0.08110878087034991+1])
	# test.append([0.1714684552570149+1, -0.00583098982142613, 0.08142222047427562+1])
	# test.append([0.06898789824234802+1, 0.015579665412308652, 0.08918678112073716+1])
	for index,element in enumerate(shelved):
		objectset,objectname=element
		obj=make_object(objectset,objectname,world)
		bb1,bb2=obj.geometry().getBB()
		obj.setTransform(so3.identity(),[-1+index*0.3,-1,0.3])
		# obj.setTransform(so3.identity(),test[index])
		# obj.setTransform(so3.identity(),[0,0,0.4])
		# print check_placement(world,obj.getTransform(),0)

	viewer = MyGLViewer(world)
	vis.run(viewer)
