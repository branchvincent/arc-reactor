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
                    target_box["drop position"]=[-0.4,0.3,0.2]
                    target_box['position']=[0.2,0.8,0.15]
                    old_config=self.sim.world.robot(0).getConfig()
                    self.trajectory=planner.pick_up(self.sim.world,target_item,target_box)
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
                    target_box["drop position"]=self.place_position[self.target]
                    target_box['position']=self.place_position[self.target]
                    print self.place_position[self.target]
                    # target_box["drop position"]=[]
                    # target_box['position']=[]
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
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    

    viewer = MyGLViewer(world)
    vis.run(viewer)
