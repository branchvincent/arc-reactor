#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.vis.glrobotprogram import *
from klampt.model import ik,coordinates,config,cartesian_trajectory,trajectory
from klampt.model.collide import WorldCollider
#from klampt.plan import cspace
from klampt.math import so3,se3
from klampt.plan.cspace import CSpace,MotionPlan
import random
import importlib
import os
import time
import json
import planner

keymap = None
ee_link=6

def build_default_keymap(world):
    """builds a default keymape: 1234567890 increases values of DOFs 1-10
    of robot 0.  qwertyuiop decreases values."""
    if world.numRobots() == 0:
        return {}
    robot = world.robot(0)
    up = '1234567890'
    down = 'qwertyuiop'
    res = {}
    for i in range(min(robot.numDrivers(),10)):
        #up velocity
        vel = [0]*robot.numLinks()
        if robot.driver(i).getType() == 'normal':
            vel[robot.driver(i).getAffectedLink()] = 1
        else:
            #skip it
            #links = robot.driver(i).getAffectedLinks();
            continue
        res[up[i]] = (0,vel)
        #down velocity
        vel = vectorops.mul(vel,-1)
        res[down[i]] = (0,vel)
    return res

# only work for cubes
# TODO: add checking for general objects
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

class MyGLViewer(GLSimulationPlugin):
    def __init__(self,world):
        global keymap
        GLSimulationPlugin.__init__(self,world)
        self.world = world
        self.robot = world.robot(0)
        self.q1 = self.robot.getConfig()
        self.robotController=self.sim.controller(0)
        if keymap == None:
            keymap = build_default_keymap(world)
        self.keymap = keymap
        self.current_velocities = {}
        self.target=world.robot(0).link(6).getWorldPosition([0,0,0])
        # self.target=vectorops.add(self.world.rigidObject(self.world.numRigidObjects()-1).getTransform()[1],[-0.05,0,0.1])
        self.numberOfObjects=world.numRigidObjects()
        self.activeObjects=False
        self.flag=0
        self.t=[]
        self.globals=Globals(world)
        self.cspace=TestCSpace(self.globals)
        #Put your initialization code here

##################################################################################################################################################################

##################################################################################################################################################################
    def IK(local_position,target_position):
        goal = ik.objective(end_link,local = local_position, world = target_position)
        if ik.solve_global(goal, 1000, 1e-6):
            #print "IK success!"
            qout = robot.getConfig()    
        else:
            #print "IK failure... returning best solution found"
            qout = robot.getConfig()
        return qout

    def control_loop(self):
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
        robot=self.world.robot(0)
        ee_link=6
        ee_local_p1=[0,0,0]
        ee_local_p3=[0.01,0,0]
        ee_world_p1=self.target
        ee_world_p3=vectorops.add(self.target,[0,0,-0.01])
        goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p3],world=[ee_world_p1,ee_world_p3])
        old_T=robot.link(ee_link).getTransform()
        old_R,old_t=old_T
        s=ik.solve_global(goal)
        q= robot.getConfig()
        q[6]=self.q1[6]
        q[7]=self.q1[7]
        q[8]=self.q1[8]
        q[9]=self.q1[9]
        q[10]=self.q1[10]
        # print self.cspace.feasible(q)
        robotController = self.sim.controller(0)
        qdes = q
        (qmin,qmax) = robot.getJointLimits()
        for i in xrange(len(qdes)-1):
            qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
        robotController.setLinear(qdes,0.1)

        return

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
        step=0.02
        cnt = 0
        GLSimulationPlugin.keyboardfunc(self,c,x,y)
        self.q1=self.robot.getConfig()
        self.refresh()
        if c == '1':
            self.target[0]+=step
        elif c== 'q':
            self.target[0]-=step
        elif c=='2':
            self.target[1]+=step
        elif c == 'w':
            self.target[1]-=step
        elif c== '3':
            self.target[2]+=step
        elif c== 'e':   
            self.target[2]-=step
        elif c=='v':
#            self.activeObjects= not self.activeObjects
            step = 0.1
            self.q1[6] += step
        elif c=='f':
#            self.activeObjects= not self.activeObjects
            step = -0.1
            self.q1[6] += step
        elif c=='a':
            step=2.1
            self.q1[7]=step
            self.q1[8]=-step
            self.q1[9]=-step
            self.q1[10]=step
            self.robotController.setMilestone(self.q1)
            print self.target
        elif c=='b':
            step=0.1
            self.q1[7]-=step
            self.q1[8]+=step
            self.q1[9]+=step
            self.q1[10]-=step
            self.robotController.setMilestone(self.q1)
            print self.q1
        elif c=='g':
            step=-0.1
            self.q1[7]-=step
            self.q1[8]+=step
            self.q1[9]+=step
            self.q1[10]-=step
            self.robotController.setMilestone(self.q1)
            print self.q1
        elif c=='x':
            step=0.37
            self.q1[7]=-step
            self.q1[8]=step
            self.q1[9]=step
            self.q1[10]=-step
            self.robotController.setMilestone(self.q1)
            print self.q1
        # elif c=='x':
        #     step=-1.5
        #     self.q1[11]=-step
        #     self.robotController.setMilestone(self.q1)
        #     print self.q1
        # elif c=='c':
        #     step=1.5
        #     self.q1[11]=-step
        #     self.robotController.setMilestone(self.q1)
        #     print self.q1
        elif c =='z':   
            step=2.1
            self.q1[7]=step
            self.q1[8]=-step
            self.q1[9]=-step
            self.q1[10]=step
            self.robotController.setMilestone(self.q1)             
            step = 0.41
            self.target[2]-=step

        else:
            GLSimulationPlugin.keyboardfunc(self,c,x,y)
            self.refresh()


    def keyboardupfunc(self,c,x,y):
        if c in self.current_velocities:
            del self.current_velocities[c]
        return


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