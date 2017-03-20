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

# box_vacuum_offset=[0.1,0,0.09]
# ee_local=[-0.015,-0.02,0.3]
# # ee_local=[0,0,0]
# box_release_offset=[0,0,0.06]
# idle_position=[0.6,0,1]
# approach_p1=[0.5,0.2,1]
# approach_p2=[0.5,-0.2,1]
# order_box_min=[0.36,0.65,0.5]
# order_box_max=[0.5278,0.904,0.5]
# angle_to_degree=57.296
ee_link=6


#shelf parameters for object rigid transformation
shelf_dims = (0.8,0.3,0.3)
shelf_x_shift = 0.4
shelf_offset = 0.45
shelf_height = 0.50
firstbin_dims = (0.20,0.20,0.25)
firstbin_x_shift = -0.09
firstbin_offset = 0.44
firstbin_height = 0.30
#load related files for objects 
object_template_fn = '../../../data/objects/object_template.obj'
objects = {}
objects['apc2017'] = [f for f in sorted(os.listdir('../../../data/objects/apc2017'))]


object_geom_file_patterns = {
	'apc2017':['../../../data/objects/apc2017/%s/mesh.ply']
}
#default mass for objects whose masses are not specified, in kg
default_object_mass = 0.5
object_masses = {
	'apc2017':dict(),
}


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

def xy_randomize(obj,bmin,bmax):
	R,t = obj.getTransform()
	obmin,obmax = obj.geometry().getBB()
	w = 0.5*(obmax[0]-obmin[0])
	h = 0.5*(obmax[1]-obmin[1])
	correction = max(w,h)
	R = so3.mul(so3.rotation([0,0,1],random.uniform(0,math.pi*2)),R)
	t[0] = random.uniform(bmin[0]+correction,bmax[0]-correction)
	t[1] = random.uniform(bmin[1]+correction,bmax[1]-correction)
	obj.setTransform(R,t)

def xy_jiggle(world,objects,fixed_objects,bmin,bmax,iters,randomize = True):
	"""Jiggles the objects' x-y positions within the range bmin - bmax, and randomizes orientation about the z
	axis until the objects are collision free.  A list of fixed objects (fixed_objects) may be given as well.

	Objects for which collision-free resolutions are not found after iters steps will be
	deleted from the world.
	"""
	if randomize:
		for obj in objects:
			xy_randomize(obj,bmin,bmax)
	inner_iters = 10
	while iters > 0:
		numConflicts = [0]*len(objects)
		for (i,j) in collide.self_collision_iter([o.geometry() for o in objects]):
			numConflicts[i] += 1
			numConflicts[j] += 1
		for (i,j) in collide.group_collision_iter([o.geometry() for o in objects],[o.geometry() for o in fixed_objects]):
			numConflicts[i] += 1
		
		amax = max((c,i) for (i,c) in enumerate(numConflicts))[1]
		cmax = numConflicts[amax]
		if cmax == 0:
			#conflict free
			return
		print cmax,"conflicts with object",objects[amax].getName()
		other_geoms = [o.geometry() for o in objects[:amax]+objects[amax+1:]+fixed_objects]
		for it in xrange(inner_iters):
			xy_randomize(objects[amax],bmin,bmax)
			nc = sum([1 for p in collide.group_collision_iter([objects[amax].geometry()],other_geoms)])
			if nc < cmax:
				break
			iters-=1
		print "Now",nc,"conflicts with object",objects[amax].getName()

	numConflicts = [0]*len(objects)
	for (i,j) in collide.self_collision_iter([o.geometry() for o in objects]):
		numConflicts[i] += 1
		numConflicts[j] += 1
	for (i,j) in collide.group_collision_iter([o.geometry() for o in objects],[o.geometry() for o in fixed_objects]):
		numConflicts[i] += 1
	removed = []
	while max(numConflicts) > 0:
		amax = max((c,i) for (i,c) in enumerate(numConflicts))[1]
		cmax = numConflicts[amax]
		print "Unable to find conflict-free configuration, removing object",objects[amax].getName(),"with",cmax,"conflicts"
		removed.append(amax)

		#revise # of conflicts -- this could be faster, but whatever...
		numConflicts = [0]*len(objects)
		for (i,j) in collide.self_collision_iter([o.geometry() for o in objects]):
			if i in removed or j in removed:
				continue
			numConflicts[i] += 1
			numConflicts[j] += 1
		for (i,j) in collide.group_collision_iter([o.geometry() for o in objects],[o.geometry() for o in fixed_objects]):
			if i in removed:
				continue
			numConflicts[i] += 1
	removeIDs = [objects[i].index for i in removed]
	for i in sorted(removeIDs)[::-1]:
		world.remove(world.rigidObject(i))
	raw_input("Press enter to continue")


class MyGLViewer(GLSimulationPlugin):
    def __init__(self,world):
        GLSimulationPlugin.__init__(self,world)
        self.world = world
        self.robotController=self.sim.controller(0)
        self.score=0
        self.total_object=world.numRigidObjects()
        self.trajectory=[]
        self.target=0
        self.t=0
        self.T=[]
        self.flag=0
        self.place_position=[]
        self.task='pick'
        self.last_end_time=0
        self.in_box=[]
        self.time_count=0
        self.old_time=0
    def control_loop(self):
        if self.sim.getTime()-self.last_end_time>0.5:
            if self.trajectory:
                if self.t<len(self.trajectory): 
                    robot=self.sim.world.robot(0)
                    old_T=robot.link(ee_link).getTransform()
                    old_R,old_t=old_T
                    self.robotController.setLinear(self.trajectory[self.t][1]['robot'],self.trajectory[self.t][0]-(self.sim.getTime()-self.old_time))
                    robot.setConfig(self.trajectory[self.t][1]['robot'])
                    obj=self.sim.world.rigidObject(self.target)
                    #get a SimBody object
                    body = self.sim.body(obj)
                    # print self.trajectory[self.t][1]['vacuum']
                    if self.trajectory[self.t][1]['simulation']==1:
                            T=body.getTransform()
                            R,t=T           
                            body.enableDynamics(False)
                            body.setVelocity((0,0,0),(0,0,0))
                            if self.flag==0:
                                self.flag=1
                                self.T=robot.link(ee_link).getLocalPosition(t)
                            # print 'here'
                            new_T=robot.link(ee_link).getTransform()
                            new_R,new_t=new_T
                            change_R=so3.mul(new_R,so3.inv(old_R))
                            
                            R=so3.mul(change_R,R)
                            body.setTransform(R,robot.link(ee_link).getWorldPosition(self.T))
                    else:
                        body.enableDynamics(True)
                        self.flag=0
                    # print 'current config', robot.getConfig()
                    # print 'goal config',self.trajectory[self.t][1]['robot']
                    # if  vectorops.distance(robot.getConfig(),self.trajectory[self.t][1]['robot'])<0.01:
                    #     print 'get there'
                    if self.sim.getTime()-self.old_time>self.trajectory[self.t][0]:
	                    self.t+=1
	                    self.old_time=self.sim.getTime()
                    if self.t==len(self.trajectory):
                        self.last_end_time=self.sim.getTime()
                else:
                    self.score+=1
                    self.in_box.append(self.target)
                    if self.task=='pick':
                        self.target+=1
                    if self.target>=self.total_object:
                        self.task='stow'
                    if self.task=='stow':
                        self.target-=1
                    print "one pick task is done!"
                    self.t=0
                    self.trajectory=[]
                    print 'score:',self.score
                    time.sleep(1)
                    if self.score==2*self.total_object:
                        print "all items are picked up!"
                        print self.sim.world.robot(0).getConfig()
                        exit()
            else:
                if self.score<self.total_object:
                    target_item={}
                    target_box={}
                    
                    if max(self.sim.world.rigidObject(self.target).getVelocity()[0])>0.01:
                        print 'turning!'
                        return
                    target_item["position"]=self.sim.world.rigidObject(self.target).getTransform()[1]
                    self.place_position.append(self.sim.world.rigidObject(self.target).getTransform()[1])
                    target_item["vacuum_offset"]=[0,0,0.1]
                    target_item["bbox"]=self.sim.world.rigidObject(self.target).geometry().getBB()
                    target_item['drop offset']=[0,0,0.2]
                    target_box["box_limit"]=[[-0.5,0.2,0.1],[-0.35,0.45,0.4]]
                    # target_box["drop position"]=[-0.4,0.3,0.2]
                    target_box["drop position"]=[]
                    target_box['position']=[0.2,0.8,0.15]
                    old_config=self.sim.world.robot(0).getConfig()
                    self.trajectory=planner.pick_up(self.sim.world,target_item,target_box,self.target)
                    if self.trajectory==False:
                        self.score+=1
                        self.target+=1
                        if self.score==self.total_object:
                            self.task='stow'
                            self.target-=1
                    else:
                    	self.old_time=self.sim.getTime()
                    	self.time_count=0
                        print "validing trajectory..."
                        max_change_joint=0
                        max_joint_speed=0
                        for i in range(len(self.trajectory)):
                            new_config=self.trajectory[i][1]['robot']
                            d_config=max(max(vectorops.sub(new_config,old_config)),-min(vectorops.sub(new_config,old_config)))
                            speed_config=d_config/self.trajectory[i][0]
                            if d_config>max_change_joint:
                                max_change_joint=d_config
                            if speed_config>max_joint_speed:
                                max_joint_speed=speed_config
                            old_config=new_config
                        print 'max joint change is :', max_change_joint/3.14159*180
                        print 'max joint speed is:', max_joint_speed/3.14159*180


                else:
                    target_item={}
                    target_box={}
                    # print self.sim.world.rigidObject(self.target).getVelocity()[0]
                    if max(self.sim.world.rigidObject(self.target).getVelocity()[0])>0.01:
                        print 'turning!'
                        return
                    target_item["position"]=self.sim.world.rigidObject(self.target).getTransform()[1]
                    target_item["vacuum_offset"]=[0,0,0.1]
                    target_item['drop offset']=[0,0,0.15]
                    target_item["bbox"]=self.sim.world.rigidObject(self.target).geometry().getBB()
                    # target_box["drop position"]=self.place_position[self.target]
                    # target_box['position']=self.place_position[self.target]
                    target_box["box_limit"]=[[-0.2,0.3,0.1],[0.6,0.5,0.4]]
                    # print self.place_position[self.target]
                    target_box["drop position"]=[]
                    target_box['position']=[]
                    old_config=self.sim.world.robot(0).getConfig()
                    self.trajectory=planner.stow(self.sim.world,target_item,target_box,self.target)
                    if self.trajectory==False:
                        self.score+=1
                        self.target-=1
                        if self.score==2*self.total_object:
                            print 'done!'
                            exit()
                    else:
                    	self.old_time=self.sim.getTime()
                    	self.time_count=0
                        print "validing trajectory..."
                        max_change_joint=0
                        max_joint_speed=0
                        for i in range(len(self.trajectory)):
                            new_config=self.trajectory[i][1]['robot']
                            d_config=max(max(vectorops.sub(new_config,old_config)),-min(vectorops.sub(new_config,old_config)))
                            speed_config=d_config/self.trajectory[i][0]
                            if d_config>max_change_joint:
                                max_change_joint=d_config
                            if speed_config>max_joint_speed:
                                max_joint_speed=speed_config
                            old_config=new_config
                        print 'max joint change is :', max_change_joint/3.14159*180
                        print 'max joint speed is:', max_joint_speed/3.14159*180

            

    


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
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode    
        # step=0.05
        # if c == 'u':
        #     self.target[0]+=step
        # elif c== 'j':
        #     self.target[0]-=step
        # elif c=='i':
        #     self.target[1]+=step
        # elif c == 'k':
        #     self.target[1]-=step
        # elif c== 'o':
        #     self.target[2]+=step
        # elif c== 'l':   
        #     self.target[2]-=step
        # elif c=='v':
        #     self.activeObjects[0]= not self.activeObjects[0]
        # elif c=='p':
        #     print 'target:',self.target

        # else:
        GLSimulationPlugin.keyboardfunc(self,c,x,y)
        self.refresh()



if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: test_planner.py ../../data/ARC_v1.xml"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:2]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)

    shelved=[]
    #set up a different translation for the first bin since the top is blocked by the robot hand effector
    firstbin=[]
    increment=0
    increment_firstbin=0
    shelf=world.terrain("shelf")
    rigid_objects=[]
    num_in_firstbin=0
    firstbin_objects=[]
    if len(sys.argv)<3:
        print "Randomly putting 3 objects in the shelf"
        for i in range(3):
            dataset=random.choice(objects.keys())
            index = random.randint(0,len(objects[dataset])-1)
	    objname = objects[dataset][index]
	    shelved.append((dataset,objname))
    
    elif len(sys.argv)==3:
        if int(sys.argv[2])/3<3:
            num_in_firstbin=int(sys.argv[2])/3
        else:
            num_in_firstbin=2

        for i in range(num_in_firstbin):
            dataset=random.choice(objects.keys())
            index = random.randint(0,len(objects[dataset])-1)
	    objname = objects[dataset][index]
	    firstbin.append((dataset,objname))
            
        for i in range(int(sys.argv[2])-num_in_firstbin):
            dataset=random.choice(objects.keys())
            index = random.randint(0,len(objects[dataset])-1)
	    objname = objects[dataset][index]
	    shelved.append((dataset,objname))
        
    for objectset,objectname in shelved:
        object=make_object(objectset,objectname,world)
        object.setTransform(*se3.mul((so3.identity(),[shelf_x_shift,shelf_offset,shelf_height+increment/2*0.15]),object.getTransform()))
        rigid_objects.append(object)
          
        increment+=1
    for objectset,objectname in firstbin:
        object=make_object(objectset,objectname,world)
        object.setTransform(*se3.mul((so3.identity(),[firstbin_x_shift,firstbin_offset,firstbin_height+increment_firstbin/2*0.22]),object.getTransform()))
        firstbin_objects.append(object)
        increment_firstbin+=1
          
    
    if len(rigid_objects)>0:
        xy_jiggle(world,rigid_objects,[shelf],[shelf_x_shift-0.5*shelf_dims[0],-0.5*shelf_dims[1]+shelf_offset],[shelf_x_shift+0.5*shelf_dims[0],0.5*shelf_dims[1]+shelf_offset],100)            
    if len(firstbin_objects)>0:
        xy_jiggle(world,firstbin_objects,[shelf],[firstbin_x_shift-0.5*firstbin_dims[0],-0.5*firstbin_dims[1]+firstbin_offset],[firstbin_x_shift+0.5*firstbin_dims[0],0.5*firstbin_dims[1]+firstbin_offset],100)            
        

    viewer = MyGLViewer(world)
    vis.run(viewer)
