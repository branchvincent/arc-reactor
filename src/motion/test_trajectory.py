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

box_vacuum_offset=[0.1,0,0.09]
ee_local=[-0.015,-0.02,0.3]
# ee_local=[0,0,0]
box_release_offset=[0,0,0.06]
idle_position=[0.6,0,1]
approach_p1=[0.5,0.2,1]
approach_p2=[0.5,-0.2,1]
order_box_min=[0.36,0.65,0.5]
order_box_max=[0.5278,0.904,0.5]
angle_to_degree=57.296


def interpolate(start_T,end_T,u,flag):
    R1,t1=start_T
    R2,t2=end_T
    T=se3.interpolate(start_T,end_T,u)
    R,t=T
    if flag:
        ik_local=[ee_local,vectorops.add(ee_local,[0.01,0,0]),vectorops.add(ee_local,[0,0,0.01])]
        ik_world=[t,vectorops.add(t,[R[0]/100,R[1]/100,R[2]/100]),vectorops.add(t,[R[6]/100,R[7]/100,R[8]/100])]
    else:
        ik_local=[ee_local,vectorops.add(ee_local,[0.01,0,0])]
        ik_world=[t,vectorops.add(t,[R[0]/100,R[1]/100,R[2]/100])]
    return [ik_local,ik_world]

#set up collision checking
class Globals:
    """docstring for Globals"""
    def __init__(self, world):
        self.world=world
        self.robot=world.robot(0)
        self.collider=WorldCollider(world)

class TestCSpace:
    def __init__(self,globals):
        self.globals=globals
        self.robot=globals.robot
        self.q0=self.robot.getConfig()
        qlimits=zip(*self.robot.getJointLimits())
        self.bound=[qlimits[i] for i in range(len(self.q0))]
        self.eps=1e-2

    def feasible(self,x):
        #check joint limit
        for (xi,bi) in zip(x,self.bound):
            if xi<bi[0] or xi>bi[1]:
                return False
        q=x[:]
        self.robot.setConfig(q)
        world=self.globals.world
        collider=self.globals.collider
        #test robot self-collision
        if any(collider.robotSelfCollisions(self.robot.index)):
            return False
        #test environment collision
        for o in xrange(world.numTerrains()):
            if any(collider.robotTerrainCollisions(self.robot.index,o)):
                return False
        return True

 
def feasible(robot,x,q_old):
        #check joint limit
        qlimits=zip(*robot.getJointLimits())
        bound=[qlimits[i] for i in range(len(qlimits))]
        for (xi,bi) in zip(x,bound):
            if xi<bi[0] or xi>bi[1]:
                print "out of joint limit!"
                return False
        max_change=0.3
        for i in range(len(x)):
            if x[i]<q_old[i]-max_change or x[i]>q_old[i]+max_change:
                print "change too much"
                return False




class MyGLViewer(GLSimulationProgram):
    def __init__(self,world):
        GLSimulationProgram.__init__(self,world,"My GL program")
        self.world = world
   

        json_data=open("test.json").read()
        self.trajectory=json.loads(json_data)
        print len(self.trajectory)
        self.t=0
        self.time=self.trajectory[0][0]
        #Put your initialization code here
    def set_state(self,state):
        print "move from", self.state, 'to ',state
        self.state=state
        self.last_state_end=self.sim.getTime()
        self.start_flag=0



    def control_loop(self):
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
        robot=self.world.robot(0)
        robotController = self.sim.controller(0)
        # q=[0,74.39887743716503, -18.14936288798376, 89.93990754620327, -105.37584356193894, 50.93074983982921, -66.42841556288458]
        # q=[0,0,0,0,0,0,90]
        # qdes=vectorops.div(q,angle_to_degree)
        # robotController.setPIDCommand(qdes,rvels[0])
        if self.t<len(self.trajectory):
            print 'sim time:',self.sim.getTime()
            print 'self.time',self.time
            if self.sim.getTime()<self.time:

                # robotController.setPIDCommand(self.trajectory[self.t][1]['robot'],rvels[0])
                robotController.setLinear(self.trajectory[self.t][1]['robot'],self.trajectory[self.t][0])
                # time.sleep(0.01)
            else:
                self.t+=1
                if self.t<len(self.trajectory):
                    self.time+=self.trajectory[self.t][0]
                
        else:
            print "done!"
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
        print "USAGE: ARC_v0.py [world_file]"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    

    viewer = MyGLViewer(world)
    viewer.run()
