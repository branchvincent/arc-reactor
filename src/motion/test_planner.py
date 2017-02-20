#!/usr/bin/python

import sys
from klampt import *
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

box_vacuum_offset=[0.1,0,0.09]
ee_local=[-0.015,-0.02,0.3]
# ee_local=[0,0,0]
box_release_offset=[0,0,0.06]
idle_position=[0.6,0,1]
approach_p1=[0.5,0.2,1]
approach_p2=[0.5,-0.1,1]
order_box_min=[0.36,0.65,0.1]
order_box_max=[0.5278,0.904,0.15]
angle_to_degree=57.296
ee_link=6
rigidObject_positions=[]
object_BB=[]

#shelf parameters for object rigid transformation
shelf_dims = (0.4,0.4,0.3)
shelf_offset = 0.45
shelf_height = 0.25
#load related files for objects 
object_template_fn = '../../..//data/objects/object_template.obj'
objects = {}
objects['ycb'] = [f for f in sorted(os.listdir('../../../data/objects/ycb'))]
objects['apc2015'] = [f for f in sorted(os.listdir('../../../data/objects/apc2015'))]


object_geom_file_patterns = {
    'ycb':['../../../data/objects/ycb/%s/meshes/tsdf_mesh.stl','~/data/objects/ycb/%s/meshes/poisson_mesh.stl'],
    'apc2015':['../../../data/objects/apc2015/%s/textured_meshes/optimized_tsdf_textured_mesh.ply']
}
#default mass for objects whose masses are not specified, in kg
default_object_mass = 0.5
object_masses = {
    'ycb':dict(),
    'apc2015':dict(),
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




class MyGLViewer(GLSimulationProgram):
    def __init__(self,world):
        GLSimulationProgram.__init__(self,world,"My GL program")
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
        self.target_list=[]
        print self.total_object

    def control_loop(self):
        if self.sim.getTime()-self.last_end_time>0.5:
            if self.trajectory:
                if self.t<len(self.trajectory): 
                    robot=self.sim.world.robot(0)
                    old_T=robot.link(ee_link).getTransform()
                    old_R,old_t=old_T 
                    self.robotController.setLinear(self.trajectory[self.t][1]['robot'],self.trajectory[self.t][0])
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
                    self.t+=1
                    if self.t==len(self.trajectory):
                        self.last_end_time=self.sim.getTime()
                else:
                    self.score+=1
                    if self.task=='pick':
                        self.target+=1
                    if self.target>=self.total_object:
                        self.task='stow'
                    if self.task=='stow':
                        self.target-=1
                    print "one pick task is done!"
                    self.t=0
                    self.trajectory=[]
                if self.score==2*self.total_object:
                    print "all items are picked up!"
                    exit()
            else:
                if self.score<self.total_object:
                    target_item={}
                    target_box={}
                    
                    if max(self.sim.world.rigidObject(self.target).getVelocity()[0])>0.05:
                        # print 'turning!'
                        return
                    target_item["position"]=self.sim.world.rigidObject(self.target).getTransform()[1]
                    target_item["bbox"]=self.sim.world.rigidObject(self.target).geometry().getBB()
                    target_item["vacuum_offset"]=[0,0,0.1]
                    target_item['drop offset']=target_item["bbox"][1][2]-target_item["bbox"][0][2]+0.2
                    target_box["drop position"]=[0.2,0.95-0.06*self.target,0.6]
                    target_box['position']=[0.2,0.8,0.05]
                    self.trajectory=planner.pick_up(self.sim.world,target_item,target_box)
                    if self.trajectory==False:
					    self.place_position.append([0,0,0])
					    print "not able to pick it!",self.target
					    self.score+=1
					    if self.task=='pick':
					    	self.target+=1
					    if self.target>=self.total_object:
					    	self.task='stow'
					    if self.task=='stow':
					    	self.target-=1
						# if self.task=='pick':
						#     self.target+=1
						# if self.target>=self.total_object:
						#     self.task='stow'
						# if self.task=='stow':
						#     self.target-=1
						# print "one pick task is done!"
                    else:
                        self.place_position.append(self.sim.world.rigidObject(self.target).getTransform()[1])
                    print self.target
                    # time.sleep(1)
                else:
                    target_item={}
                    target_box={}
                    # print self.sim.world.rigidObject(self.target).getVelocity()[0]
                    
                    if self.target<0 or not self.place_position[self.target]:
                        self.target-=1
                        if self.target>=0:
                            return
                        else:
                            print "Done!"
                            exit()
                    if max(self.sim.world.rigidObject(self.target).getVelocity()[0])>0.001:
                        # print 'turning!'
                        return
                    print 'stowing:',self.target
                    target_item["position"]=self.sim.world.rigidObject(self.target).getTransform()[1]
                    target_item["bbox"]=self.sim.world.rigidObject(self.target).geometry().getBB()
                    target_item["vacuum_offset"]=[0,0,0.1]
                    target_item['drop offset']=target_item["bbox"][1][2]-target_item["bbox"][0][2]+0.2
                    target_box["drop position"]=self.place_position[self.target]
                    target_box['position']=self.place_position[self.target]
                    self.trajectory=planner.stow(self.sim.world,target_item,target_box)
                    if self.trajectory==False:
                        print "not able to pick it!"
                        self.score+=1
                        if self.task=='stow':
                            self.target-=1
                        if self.score==2*self.total_object:
                            print "Done!"
                            exit()

            

    


    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        if button==2:
            if state==0:
                print [o.getName() for o in self.click_world(x,y)]
                return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def print_help(self):
        GLSimulationProgram.print_help(self)
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
        GLSimulationProgram.keyboardfunc(self,c,x,y)
        self.refresh()



if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "USAGE: test_planner.py ../../data/ARC_v1.xml"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    # shelved=[]
    # increment=0
    # shelf=world.terrain("shelf")
    # rigid_objects=[]
    # for i in range(2):
    #     dataset=random.choice(objects.keys())
    #     index = random.randint(0,len(objects[dataset])-1)
    #     objname = objects[dataset][index]
    #     shelved.append((dataset,objname))
    #     for objectset,objectname in shelved:
    #         object=make_object(objectset,objectname,world)
    #         object.setTransform(*se3.mul((so3.identity(),[1,shelf_offset,shelf_height+increment/2*0.25]),object.getTransform()))
    #         rigid_objects.append(object)
          
    #         increment+=1
    #     xy_jiggle(world,rigid_objects,[shelf],[1-0.5*shelf_dims[0],-0.5*shelf_dims[1]+shelf_offset],[1+0.5*shelf_dims[0],0.5*shelf_dims[1]+shelf_offset],100)
    #     """    
    #     rigidObject_positions.append([1-increment%2*0.04,shelf_offset+0.01*increment+increment%2*0.09,shelf_height+increment/2*0.25])
    #     object_BB.append(object.geometry().getBB())
    #     """

    viewer = MyGLViewer(world)
    viewer.run()
