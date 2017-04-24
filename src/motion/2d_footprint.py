#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.vis.gldraw import *
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
import numpy as np
from scipy import signal
from klampt.model import sensing
try:
	import matplotlib.pyplot as plt
	HAVE_PYPLOT = True
except ImportError:
	HAVE_PYPLOT = False
	print "**** Matplotlib not available, can't plot color/depth images ***"

plot_flag=1

order_box_name=['A1','1AD','1A5','1B2','K3']
number_of_item=[2,2,3,3,5]

# order_box_name=['K3']
# number_of_item=[5]
def processDepthSensor(sensor):
	rgb,depth = sensing.camera_to_images(sensor)
	return rgb,depth



box_limit={}
# box_limit['1A5']=[[-0.1715,-0.1395,0.05],[0.1715,0.1395,0.121]]
box_limit['A1']=[[-0.127,-0.0889,0],[0.127,0.0889,0.08255]]
box_limit['1AD']=[[-0.1515,-0.1005,0.03],[0.1615,0.1005,0.089]]
box_limit['1A5']=[[-0.1515,-0.1195,0.03],[0.1615,0.1195,0.121]]
box_limit['1B2']=[[-0.177,-0.145,0.03],[0.187,0.145,0.089]]
box_limit['K3']=[[-0.2275,-0.145,0.03],[0.2375,0.145,0.159]]
#load related files for objects 
object_template_fn = '../../data/objects/object_template.obj'
objects = {}
real_item=0

if real_item:
	objects['apc2017'] = [f for f in sorted(os.listdir('../../data/objects/apc2017'))]
	if 'gen_thumbnail' in objects['apc2017']:
		objects['apc2017'].remove("gen_thumbnail")
	if 'convert_psd' in objects['apc2017']:
		objects['apc2017'].remove("convert_psd")
	if 'Composition_Book' in objects['apc2017']:
		objects['apc2017'].remove("Composition_Book")
	object_geom_file_patterns = {
		'apc2017':['../../data/objects/apc2017/%s/mesh.ply']
	}
else:
	objects['apc2017'] = [f for f in sorted(os.listdir('../../data/objects/apc2017_cube'))]

	if 'gen_thumbnail' in objects['apc2017']:
		objects['apc2017'].remove("gen_thumbnail")
	if 'convert_psd' in objects['apc2017']:
		objects['apc2017'].remove("convert_psd")
	if 'Composition_Book' in objects['apc2017']:
		objects['apc2017'].remove("Composition_Book")
	# if 'Avery_Binder' in objects['apc2017']:
	# 	objects['apc2017'].remove("Avery_Binder")
		

	object_geom_file_patterns = {
		'apc2017':['../../data/objects/apc2017_cube/%s/BB.stl']
	}
#default mass for objects whose masses are not specified, in kg
default_object_mass = 0.5
object_masses = {
	'apc2017':dict(),
}


def generate_orders(n,box,box_fit_item,box_volume,item_dictionary,box_name,utilization_rate=0):
	# utilization_rate is the total item volume/box volume. range is (0,1). Higher value means the items are taking much space of the box
		fitting_item=box_fit_item[box]
		if 'composition_book' in fitting_item:
			fitting_item.remove('composition_book')
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
				if fitting_item[item]=='composition_book':
					item_v=item_v+box_v
				item_v=item_v+item_dictionary[fitting_item[item]]
			count+=1

			if count>50:
				flag=1
				break
		if flag:
			print ("timeout! can't reach the utilization rate with %d items",n)
		else:
			# print "size_id:",box_name[box]
			# print "item volume/box volume:",item_v/box_v*100
			order_list=[]
			for item in items:
				order_list.append(fitting_item[item])
			return order_list,item_v/box_v*100


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
		# print "terrain!"
		# print i
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

def world_to_image_index(camera,robot,world_p):
	w = int(camera.getSetting('xres'))
	h = int(camera.getSetting('yres'))
	xfov = float(camera.getSetting('xfov'))
	yfov=1.0*xfov/w*h
	# print xfov,yfov
	Tcamera=sensing.get_sensor_xform(camera)
	x,y,z=se3.apply(Tcamera,world_p)
	if z<=0 or abs(x/z)>(xfov/2.0) or abs(y/z)>(yfov/2.0):
		print "out of range!"
		return False
	else:
		index_x=int((x/z)*w*0.5/math.tan(xfov*0.5)+w*0.5)
		index_y=int((y/z)*h*0.5/math.tan(yfov*0.5)+h*0.5)
		# print index_x,index_y
		return [index_x,index_y]
def image_to_world(camera,robot,image_index):
	w = int(camera.getSetting('xres'))
	h = int(camera.getSetting('yres'))
	xfov = float(camera.getSetting('xfov'))
	yfov=1.0*xfov/w*h
	# print xfov,yfov
	Tcamera=sensing.get_sensor_xform(camera)
	height=Tcamera[1][2]
	scale = math.tan(xfov*0.5)/(w*0.5)
	xshift = -w*0.5
	yshift = -h*0.5
	x,y=image_index
	temp_x=(x+xshift)*scale*height
	temp_y=(y+yshift)*scale*height

	real_x,real_y,real_z=se3.apply(se3.inv(Tcamera),[temp_x,temp_y,height])
	return [real_x,real_y]




class MyGLViewer(GLSimulationPlugin):
	def __init__(self,world):
		GLSimulationPlugin.__init__(self,world)
		self.world = world
		self.robot=world.robot(0)
		self.target=0
		self.reset_T=[]
		self.start_time=0
		self.packing_mode=1
		self.order_box_index=4
		self.box_limit=box_limit[order_box_name[self.order_box_index]]
		self.score0=[]
		self.score1=[]
		self.h0=[]
		self.h1=[]
		self.flag=0
		self.count=0
		self.order_record=[]
		self.volume_rate=[]
		self.placement0=[]
		self.placement1=[]
		self.sensor = self.sim.controller(0).sensor("rgbd_camera")
		T = (so3.mul(so3.rotation([1,0,0],math.radians(-90)),[1,0,0, 0,0,-1,  0,1,0]),[0,0.0,1.1])
		sensing.set_sensor_xform(self.sensor,T,link=-1)
		# print self.world.terrain("order_box").geometry().getBB()
		for i in range(self.world.numRigidObjects()):
			self.sim.body(self.sim.world.rigidObject(i)).enable(False)
			self.reset_T.append(self.world.rigidObject(i).getTransform())
			# print self.world.rigidObject(i).getTransform()[1]
		self.rgb=None
		self.depth=None
		self.convolution_result=None
		self.box_depth=None
		self.temp_plot=[0,0,1]


		def plot_depth():
			self.rgb,self.depth = processDepthSensor(self.sensor)
			print '----------------------------'
			bb0,bb1=box_limit['K3']
			bb0[2]=bb1[2]
			x1,y1=world_to_image_index(self.sensor,self.robot,bb0)
			x2,y2=world_to_image_index(self.sensor,self.robot,bb1)
			print len(self.depth)
			self.box_depth=[]
			for i in xrange(y2,y1+1):
				self.box_depth.append(self.depth[i][x1:x2+1])
			if self.depth is not None:
				plt.imshow(self.box_depth)
				plt.plot(self.temp_plot[0],self.temp_plot[1],'r*')
				plt.show()
	
		def plot_convolution():
			self.rgb,self.depth = processDepthSensor(self.sensor)
			bb0,bb1=box_limit['K3']
			bb0[2]=bb1[2]
			x1,y1=world_to_image_index(self.sensor,self.robot,bb0)
			x2,y2=world_to_image_index(self.sensor,self.robot,bb1)
			# print len(self.depth)
			self.box_depth=[]
			for i in xrange(y2,y1+1):
				self.box_depth.append(self.depth[i][x1:x2+1])
			bmin,bmax = self.world.rigidObject(self.target).geometry().getBB()
			dx=bmax[0]-bmin[0]
			dy=bmax[1]-bmin[1]
			mask=np.ones((int(dy/0.35*128),int(dx/0.7*256)))
			result=signal.convolve2d(self.box_depth,mask,mode='same')
			temp_data=np.array(result)
			print np.argmax(temp_data)

			if self.box_depth is not None:
				plt.imshow(result)
				plt.show()
		if HAVE_PYPLOT:
			self.add_action(plot_depth,'Plot depth','d')
			self.add_action(plot_convolution,'Plot convolution','f')
	def control_loop(self):
		#run n tests for each box, until self.count>n. Here n=3
		if self.packing_mode==0 and self.sim.getTime()>self.start_time+1+1.5*self.target:
			if self.target<self.world.numRigidObjects():
				print "pacing the object!"
				self.world.rigidObject(self.target).setTransform(so3.identity(),[0,0,0.4])
				bmin,bmax = self.world.rigidObject(self.target).geometry().getBB()
				com=vectorops.div(vectorops.add(bmin,bmax),2.0)
				obj=self.sim.body(self.world.rigidObject(self.target))
				good_t=[-(bmin[0]+bmax[0])*0.5,-(bmin[1]+bmax[1])*0.5,self.box_limit[1][2]+0.25]
				good_R=so3.identity()
				for theta in range(4):
					self.world.rigidObject(self.target).setTransform(so3.rotation((0,0,1),math.radians(90*theta)),[0,0,0.5])
					bmin,bmax = self.world.rigidObject(self.target).geometry().getBB()
					com_temp=vectorops.div(vectorops.add(bmin,bmax),2.0)
					offset=vectorops.sub([0,0,0.5],com_temp)
					for i in range(10):
						R,t=self.random_feasible_packing(offset)
						self.world.rigidObject(self.target).setTransform(R,t)
						bmin,bmax = self.world.rigidObject(self.target).geometry().getBB()
						com_temp=vectorops.div(vectorops.add(bmin,bmax),2.0)
						if com_temp[2]<com[2]:
							# print "better position!"
							com=com_temp
							good_t=t
							good_R=R
						elif com_temp[2]<com[2]+0.001 and com_temp[0]<com[0]:
							com=com_temp
							good_t=t
							good_R=R
				print 'final choice'
				print good_t
				self.placement0.append([good_R,good_t])
				obj.setObjectTransform(good_R,good_t)
				# obj.enable(True)
				self.target+=1
			else:
				self.make_score()
				self.reset_items()
				self.packing_mode=1
		elif self.packing_mode==1 and self.sim.getTime()>self.start_time+1+1.5*self.target:
			if self.target<self.world.numRigidObjects():
				print "pacing the object!"
				self.world.rigidObject(self.target).setTransform(so3.identity(),[0,0,0.4])
				bmin,bmax = self.world.rigidObject(self.target).geometry().getBB()
				com=vectorops.div(vectorops.add(bmin,bmax),2.0)
				obj=self.sim.body(self.world.rigidObject(self.target))
				good_t=[-(bmin[0]+bmax[0])*0.5,-(bmin[1]+bmax[1])*0.5,self.box_limit[1][2]+0.25]
				good_R=so3.identity()
				for theta in range(4):
					self.world.rigidObject(self.target).setTransform(so3.rotation((0,0,1),math.radians(90*theta)),[0,0,0.5])
					bmin,bmax = self.world.rigidObject(self.target).geometry().getBB()
					com_temp=vectorops.div(vectorops.add(bmin,bmax),2.0)
					offset=vectorops.sub([0,0,0.5],com_temp)
					# print 'offset',offset
					# print com_temp
					convolve_result=self.do_convolve()
					for i in range(10):
						if i==0 and theta==0:
							plot_flag=1
						else:
							plot_flag=0
						R,t=self.footprint_feasible_packing(offset,convolve_result)
						self.world.rigidObject(self.target).setTransform(R,t)
						bmin,bmax = self.world.rigidObject(self.target).geometry().getBB()
						com_temp=vectorops.div(vectorops.add(bmin,bmax),2.0)
						if com_temp[2]<com[2]:
							# print "better position!"
							com=com_temp
							good_t=t
							good_R=R
						elif com_temp[2]<com[2]+0.001 and com_temp[0]<com[0]:
							com=com_temp
							good_t=t
							good_R=R
				print 'final choice'
				print good_t
				self.placement1.append([good_R,good_t])
				obj.setObjectTransform(good_R,good_t)
				# obj.enable(True)
				self.target+=1
			else:
				self.make_score()
				# self.reset_items()
				self.packing_mode=2
		elif self.packing_mode==2:
			# obj=self.sim.body(self.world.rigidObject(0))

			# obj.enable(True)
			print 'score0:',self.score0,'score1:',self.score1
			print 'h0:',self.h0,'h1:',self.h1
			# print self.h1
			pass

	def reset_items(self):
		for i in range(self.world.numRigidObjects()):	
			R,t=self.reset_T[i]
			# print t
			obj=self.sim.body(self.world.rigidObject(i))
			obj.setObjectTransform(R,t)
			obj.enable(False)
			self.target=0
			self.start_time=self.sim.getTime()

	def draw_box(self,p):
		s=[0.01,0.01,0.01]
		box(vectorops.sub(p,s),vectorops.add(p,s))


	def display(self):
		self.sim.drawGL()
		bb=self.world.rigidObject(0).geometry().getBB()
		# bb=self.world.terrain("order_box").geometry().getBB()
		# bb=box_limit['K3']
		box(bb[0],bb[1],filled=False)
		# self.draw_box(self.temp_plot)

        #draw points on the robot
        # lh = Hand('l')
        # rh = Hand('r')
        # glDisable(GL_LIGHTING)
        # glPointSize(5.0)
        # glDisable(GL_DEPTH_TEST)
        # glBegin(GL_POINTS)
        # glColor3f(0,1,0)
        # glVertex3fv(se3.apply(self.world.robot(0).link(lh.link).getTransform(),lh.localPosition1))
        # glVertex3fv(se3.apply(self.world.robot(0).link(rh.link).getTransform(),rh.localPosition1))
        # glColor3f(0,0,1)
        # glVertex3fv(se3.apply(self.world.robot(0).link(lh.link).getTransform(),lh.localPosition2))
        # glVertex3fv(se3.apply(self.world.robot(0).link(rh.link).getTransform(),rh.localPosition2))
        # glColor3f(1,0,0)
        # glVertex3fv(self.world.rigidObject(0).getTransform()[1])
        # glEnd()
        # glEnable(GL_DEPTH_TEST)

	def make_score(self):
		score=0
		highest=0
		for i in range(self.world.numRigidObjects()):
			bb1,bb2= self.world.rigidObject(i).geometry().getBB()
			# print '---------------------'
			# print 'bb1:',bb1
			# print 'bb2:',bb2
			# print '---------------------'
			t=vectorops.div(vectorops.add(bb1,bb2),2.0)
			h=bb2[2]
			if t[0]>self.box_limit[0][0] and t[0]<self.box_limit[1][0] and t[1]>self.box_limit[0][1] and t[1]<self.box_limit[1][1]:
				score+=10
			if h>highest:
				highest=h
		# print 'highest point is',highest
		if score==10*self.world.numRigidObjects():
			if highest<self.box_limit[1][2]:
				score+=10
		if self.packing_mode==0:
			self.score0.append(score)
			self.h0.append(highest)
		elif self.packing_mode==1:
			self.score1.append(score)
			self.h1.append(highest)
	def do_convolve(self):
		self.rgb,self.depth = processDepthSensor(self.sensor)
		bb0,bb1=box_limit['K3']
		temp0=[bb0[0],bb0[1],bb1[2]]
		temp1=bb1
		x1,y1=world_to_image_index(self.sensor,self.robot,temp0)
		x2,y2=world_to_image_index(self.sensor,self.robot,temp1)
		# print len(self.depth)
		self.box_depth=[]
		for i in xrange(y2,y1+1):
			self.box_depth.append(self.depth[i][x1:x2+1])
		bmin,bmax = self.world.rigidObject(self.target).geometry().getBB()
		dx=bmax[0]-bmin[0]
		dy=bmax[1]-bmin[1]
		mask=np.ones((int(dy/0.35*128),int(dx/0.7*256)))
		result=signal.convolve2d(self.box_depth,mask,mode='same')
		temp_data=np.array(result)
		temp_data=np.reshape(temp_data,-1)
		temp_data=np.divide(temp_data,np.max(temp_data))
		temp_data[temp_data<0.99]=0
		# print max(temp_data)
		temp_p=np.divide(temp_data,np.sum(temp_data)*1.0)
		# print min(temp_data)
		temp_w=x2-x1+1
		temp_h=y1-y2+1
		output={}
		output['temp_w']=temp_w
		output['temp_h']=temp_h
		output['temp_p']=temp_p
		output['x1']=x1
		output['y2']=y2
		output['result']=result
		return output

	def footprint_helper(self,convolve_result,box_max):
		temp_w=convolve_result['temp_w']
		temp_h=convolve_result['temp_h']
		temp_p=convolve_result['temp_p']
		x1=convolve_result['x1']
		y2=convolve_result['y2']
		result=convolve_result['result']
		# max_index=np.argmax(temp_data)
		max_index=np.random.choice(len(temp_p),p=temp_p)
		# print 'temp_w',temp_w,'temp_h',temp_h
		# print max_index
		temp_y=int(max_index/temp_w)
		temp_x=int(max_index-temp_y*temp_w)
		# print 'temp_x',temp_x
		# print "temp_y",temp_y
		if plot_flag and self.target>3 and self.target<6:
			plt.imshow(result)
			plt.plot(temp_x,temp_y,'b*')
			plt.show()
		
		x=temp_x+x1
		y=temp_y+y2
		# print 'x',x,'y',y	
		real_x,real_y=image_to_world(self.sensor,self.robot,[x,y])
		return [real_x,real_y,box_max[2]]
	def footprint_feasible_packing(self,offset,convolve_result):
		box_min,box_max=self.box_limit
		box_min=vectorops.add(box_min,offset)
		box_max=vectorops.add(box_max,offset)
		target_index=self.target
		origin_T=self.world.rigidObject(target_index).getTransform()
		goal_T=[[],[]]
		goal_T[0]=origin_T[0]
		flag=1
		n=0
		
		while (flag and n<3):
			x,y,z=self.footprint_helper(convolve_result,box_max)
			# print x,y,z
			# self.temp_plot=[x,y,z]
			goal_T[1]=vectorops.add([x,y,z],offset)
			if check_placement(self.world,goal_T,target_index,self.box_limit):
				flag=0
				drop_position=goal_T[1]
			n+=1
		if flag:
			print "can't find a feasible placement"
			self.world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
			return [origin_T[0],origin_T[1]]
		# as low as possible
		[x,y,z]=drop_position
		low_bound=0
		high_bound=z
		n=0
		# print 'before lowering',drop_position
		while n<5:
			new_z=0.5*(low_bound+high_bound)
			goal_T[1]=[x,y,new_z]
			# print target_index
			if check_placement(self.world,goal_T,target_index,self.box_limit):
				drop_position=goal_T[1]
				high_bound=new_z
				# print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		# print 'after lowering z',drop_position
		# move to minus x direction
		[x,y,z]=drop_position
		low_bound=box_min[0]
		high_bound=x
		n=0
		while n<5:
			new_x=0.5*(low_bound+high_bound)
			goal_T[1]=[new_x,y,z]
			if check_placement(self.world,goal_T,target_index,self.box_limit):
				drop_position=goal_T[1]
				high_bound=new_x
				# print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		#move to minus y direction
		[x,y,z]=drop_position
		low_bound=box_min[1]
		high_bound=y
		n=0
		while n<5:
			new_y=0.5*(low_bound+high_bound)
			goal_T[1]=[x,new_y,z]
			if check_placement(self.world,goal_T,target_index,self.box_limit):
				drop_position=goal_T[1]
				high_bound=new_y
				# print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		self.world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
		# print 'the lowest positin is',drop_position
		return [origin_T[0],drop_position]
	def random_feasible_packing(self,offset):
		box_min,box_max=self.box_limit
		box_min=vectorops.add(box_min,offset)
		box_max=vectorops.add(box_max,offset)
		target_index=self.target
		origin_T=self.world.rigidObject(target_index).getTransform()
		goal_T=[[],[]]
		goal_T[0]=origin_T[0]
		flag=1
		n=0
		while (flag and n<10):
			x=random.uniform(box_min[0],box_max[0])
			y=random.uniform(box_min[1],box_max[1])
			z=random.uniform(box_min[2],box_max[2])
			goal_T[1]=[x,y,z]
			if check_placement(self.world,goal_T,target_index,self.box_limit):
				flag=0
				drop_position=goal_T[1]
			n+=1
		if flag:
			# print "can't find a feasible placement"
			self.world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
			return [origin_T[0],origin_T[1]]
		
		#as low as possible
		drop_position=[0,0,0.2]
		[x,y,z]=drop_position
		low_bound=0
		high_bound=z
		n=0
		while n<5:
			new_z=0.5*(low_bound+high_bound)
			goal_T[1]=[x,y,new_z]
			if check_placement(self.world,goal_T,target_index,self.box_limit):
				drop_position=goal_T[1]
				high_bound=new_z
				# print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		#move to minus x direction
		# print drop_position
		[x,y,z]=drop_position
		low_bound=box_min[0]
		high_bound=x
		n=0
		while n<5:
			new_x=0.5*(low_bound+high_bound)
			goal_T[1]=[new_x,y,z]
			if check_placement(self.world,goal_T,target_index,self.box_limit):
				drop_position=goal_T[1]
				high_bound=new_x
				# print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		#move to minus y direction
		[x,y,z]=drop_position
		low_bound=box_min[1]
		high_bound=y
		n=0
		while n<5:
			new_y=0.5*(low_bound+high_bound)
			goal_T[1]=[x,new_y,z]
			if check_placement(self.world,goal_T,target_index,self.box_limit):
				drop_position=goal_T[1]
				high_bound=new_y
				# print 'lower!!'
			else:
				low_bound=0.5*(low_bound+high_bound)
			n+=1
		self.world.rigidObject(target_index).setTransform(origin_T[0],origin_T[1])
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
			print check_placement(self.world,self.world.rigidObject(0).getTransform(),0,self.box_limit)
			obj=self.sim.body(self.world.rigidObject(0))
			obj.enable(True)
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
	
	# with open("pick_task_generator.json") as json_file:
	# 		data=json.load(json_file)
	# box_fit_item=data['box_fit_item']
	# box_volume=data['box_volume']
	# item_dictionary=data['item_dictionary']
	# box_name=data['box_name']	
	n=8
	order_box=1
	with open("pick_task_generator.json") as json_file:
		data=json.load(json_file)
		box_fit_item=data['box_fit_item']
		box_volume=data['box_volume']
		item_dictionary=data['item_dictionary']
		box_name=data['box_name']	
		shelved=[]
		order_list,v_rate=generate_orders(n,order_box,box_fit_item,box_volume,item_dictionary,box_name,0.3)
		print order_list
	shelved=[]
	# order_box=2
	# n=5
	# order_list=generate_orders(n,order_box,box_fit_item,box_volume,item_dictionary,box_name,0.2)
	# name_index={}
	# for i in range(40):
	# 	name_index[box_fit_item[4][i]]=i
	# # print order_list
	# order_list=['duct_tape','crayons','burts_bees_baby_wipes','ice_cube_tray','tissue_box']
	name_index={}
			# for i in range(len(box_fit_item[4])):
			# 	name_index[box_fit_item[4][i]]=i
	for i in range(len(objects['apc2017'])):
		name_index[objects['apc2017'][i].lower()]=i
	# print name_index
	for i in range(n):
			dataset=random.choice(objects.keys())
			# index = random.randint(0,len(objects[dataset])-1)
			index=name_index[order_list[i]]
			# print index
			# index=3
			objname = objects[dataset][index]

			# print dataset,objname
			shelved.append((dataset,objname))

	for index,element in enumerate(shelved):
		objectset,objectname=element
		obj=make_object(objectset,objectname,world)
		bb1,bb2=obj.geometry().getBB()
		obj.setTransform(so3.identity(),[-1+index*0.5,-1,0.3])








	# for i in range(5):
	# 		dataset=random.choice(objects.keys())
	# 		index = random.randint(0,len(objects[dataset])-1)
	# 		# index=3
	# 		# while index==7:
	# 		# 	index = random.randint(0,len(objects[dataset])-1)
	# 		# index=i+3
	# 		objname = objects[dataset][index]
	# 		print objname
	# 		# print dataset,objname
	# 		shelved.append((dataset,objname))

	# # # test=[]
	# # # test.append([-0.0468572843527858+1, 0.01388161073849592, 0.08110878087034991+1])
	# # # test.append([0.1714684552570149+1, -0.00583098982142613, 0.08142222047427562+1])
	# # # test.append([0.06898789824234802+1, 0.015579665412308652, 0.08918678112073716+1])
	# for index,element in enumerate(shelved):
	# 	objectset,objectname=element
	# 	obj=make_object(objectset,objectname,world)
	# 	bb1,bb2=obj.geometry().getBB()
	# 	obj.setTransform(so3.rotation((0,0,1),math.radians(0)),[-1+index*0.3,-1,0.3])
		# obj.setTransform(so3.identity(),test[index])
		# obj.setTransform(so3.identity(),[0,0,0.4])
		# print check_placement(world,obj.getTransform(),0)

	viewer = MyGLViewer(world)
	vis.run(viewer)
