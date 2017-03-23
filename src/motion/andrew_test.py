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
        self.robot = world.robot(0)
        self.q = self.robot.getConfig()
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

##################################################################################################################################################################

##################################################################################################################################################################

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
        #q = self.robot.getConfig()
        step=0.11

        # else:
        GLSimulationPlugin.keyboardfunc(self,c,x,y)
        self.q=self.robot.getConfig()
        self.refresh()
        if c == 'u':
            self.q[2]+=step
            self.robotController.setMilestone(self.q)
        elif c== 'j':
            self.q[2]-=step
            self.robotController.setMilestone(self.q)
        elif c=='i':
            self.q[3]+=step
            self.robotController.setMilestone(self.q)
        elif c == 'k':
            self.q[3]-=step
            self.robotController.setMilestone(self.q)
        elif c== 'o':
            self.q[8]+=step
            self.q[9]-=step
            self.q[10]-=step
            self.q[11]+=step
            self.robotController.setMilestone(self.q)
        elif c== 'l':   
            self.q[8]-=step
            self.q[9]+=step
            self.q[10]+=step
            self.q[11]-=step
            self.robotController.setMilestone(self.q)
        elif c=='a':
            step=2.44
            self.q[8]=step
            self.q[9]=-step
            self.q[10]=-step
            self.q[11]=step
            self.robotController.setMilestone(self.q)
        elif c=='b':
            step=0
            self.q[8]=-step
            self.q[9]=step
            self.q[10]=step
            self.q[11]=-step
            self.robotController.setMilestone(self.q)
        elif c=='v':
            self.q[12]+=step
            self.robotController.setMilestone(self.q)
        elif c=='p':
            print 'robot config:',self.robot.getConfig()    
        else:
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
