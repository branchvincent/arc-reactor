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
ee_link=7
time1 = 0
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
        self.numberOfObjects=world.numRigidObjects()
        self.activeObjects=False
        self.flag=0
        self.t=[]
        self.globals=Globals(world)
        self.cspace=TestCSpace(self.globals)

        #Put your initialization code here

##################################################################################################################################################################

##################################################################################################################################################################
    def control_loop(self):
            

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
        qinit = self.robot.getConfig()
        step=0.02
        cnt = 0
        GLSimulationPlugin.keyboardfunc(self,c,x,y)
        self.q1=self.robot.getConfig()
        self.refresh()



        if c == 'z':

            # 1. Open Gripper
            step=2.1
            self.q1[7]=step
            self.q1[8]=-step
            self.q1[9]=-step
            self.q1[10]=step
            self.robotController.setLinear(self.q1,2)

            # 2. Calculate IK and move to first target
            offset=[0,0,0.22] #use 0.22 for cubes
            topValue = [0,0,0.62]
            self.objectPos = vectorops.add(self.world.rigidObject(0).getTransform()[1],offset)
            self.topPosition = vectorops.add(self.world.rigidObject(0).getTransform()[1],topValue)
            self.objectOrientation = self.world.rigidObject(0).getTransform()[0]
            self.target[0] = self.objectPos[0]
            self.target[1] = self.objectPos[1]
            self.target[2] = self.objectPos[2]
            #Calculate the desired velocity for each robot by adding up all
            #commands
            rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
            robot=self.world.robot(0)
            ee_link=6
            ee_local_p1=[0,0,0]
            ee_local_p2=[0,-0.01,0]
            ee_local_p3=[0.01,0,0]
            ee_world_p1=self.target
            #you need to find the desired direction for each item, here I'm just setting it to align with either world x or y axis
            ee_world_p2=vectorops.add(self.target,[0,0.01,0])
            ee_world_p3=vectorops.add(self.target,[0,0,-0.01])
            goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p2,ee_local_p3],world=[ee_world_p1,ee_world_p2,ee_world_p3])
            #goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p3],world=[ee_world_p1,ee_world_p3])
            s=ik.solve_global(goal)
            q= robot.getConfig()
            q[7]=self.q1[7]
            q[8]=self.q1[8]
            q[9]=self.q1[9]
            q[10]=self.q1[10]
            robotController = self.sim.controller(0)
            qdes = q
            (qmin,qmax) = robot.getJointLimits()
            for i in xrange(len(qdes)-1):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.addLinear(qdes,1.1)

            # 3. Close the grippers
            #step=1.7
            step = 1.9
            self.q1[7]-=step
            self.q1[8]+=step
            self.q1[9]+=step
            self.q1[10]-=step
            q[7]=self.q1[7]
            q[8]=self.q1[8]
            q[9]=self.q1[9]
            q[10]=self.q1[10]
            robotController = self.sim.controller(0)
            qdes = q
            (qmin,qmax) = robot.getJointLimits()
            for i in xrange(len(qdes)-1):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.addLinear(qdes,1.1)

            # 4. Pick up the object (Move upwards)
            self.target[0] = self.topPosition[0]
            self.target[1] = self.topPosition[1]
            self.target[2] = self.topPosition[2]
            #Calculate the desired velocity for each robot by adding up all
            #commands
            rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
            robot=self.world.robot(0)
            ee_link=6
            ee_local_p1=[0,0,0]
            ee_local_p2=[0,-0.01,0]
            ee_local_p3=[0.01,0,0]
            ee_world_p1=self.target
            #you need to find the desired direction for each item, here I'm just setting it to align with either world x or y axis
            ee_world_p2=vectorops.add(self.target,[0,0.01,0])
            ee_world_p3=vectorops.add(self.target,[0,0,-0.01])
            goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p2,ee_local_p3],world=[ee_world_p1,ee_world_p2,ee_world_p3])
            #goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p3],world=[ee_world_p1,ee_world_p3])
            s=ik.solve_global(goal)
            q= robot.getConfig()
            q[7]=self.q1[7]
            q[8]=self.q1[8]
            q[9]=self.q1[9]
            q[10]=self.q1[10]
            robotController = self.sim.controller(0)
            qdes = q
            (qmin,qmax) = robot.getJointLimits()
            for i in xrange(len(qdes)-1):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.addLinear(qdes,1.1)

            # 5. Move to the bin
            #self.binPos = [-0.45,0.4,0.62]
            self.binPos = [0.15,0.5,0.62]
            self.target[0] = self.binPos[0]
            self.target[1] = self.binPos[1]
            self.target[2] = self.binPos[2]

            #Calculate the desired velocity for each robot by adding up all
            #commands
            rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
            robot=self.world.robot(0)
            ee_link=6
            ee_local_p1=[0,0,0]
            ee_local_p2=[0,-0.01,0]
            ee_local_p3=[0.01,0,0]
            ee_world_p1=self.target
            #you need to find the desired direction for each item, here I'm just setting it to align with either world x or y axis
            ee_world_p2=vectorops.add(self.target,[0,0.01,0])
            ee_world_p3=vectorops.add(self.target,[0,0,-0.01])
            goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p2,ee_local_p3],world=[ee_world_p1,ee_world_p2,ee_world_p3])
            #goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p3],world=[ee_world_p1,ee_world_p3])
            s=ik.solve_global(goal)
            q= robot.getConfig()
            q[7]=self.q1[7]
            q[8]=self.q1[8]
            q[9]=self.q1[9]
            q[10]=self.q1[10]
            robotController = self.sim.controller(0)
            qdes = q
            (qmin,qmax) = robot.getJointLimits()
            for i in xrange(len(qdes)-1):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.addLinear(qdes,1.1)

            # 6. Go down to the bin
            #self.binDown = [-0.45,0.4,0.32]
            self.binDown = [0.15,0.5,0.32]
            self.target[0] = self.binDown[0]
            self.target[1] = self.binDown[1]
            self.target[2] = self.binDown[2]

            #Calculate the desired velocity for each robot by adding up all
            #commands
            rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
            robot=self.world.robot(0)
            ee_link=6
            ee_local_p1=[0,0,0]
            ee_local_p2=[0,-0.01,0]
            ee_local_p3=[0.01,0,0]
            ee_world_p1=self.target
            #you need to find the desired direction for each item, here I'm just setting it to align with either world x or y axis
            ee_world_p2=vectorops.add(self.target,[0,0.01,0])
            ee_world_p3=vectorops.add(self.target,[0,0,-0.01])
            goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p2,ee_local_p3],world=[ee_world_p1,ee_world_p2,ee_world_p3])
            #goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p3],world=[ee_world_p1,ee_world_p3])
            s=ik.solve_global(goal)
            q= robot.getConfig()
            q[7]=self.q1[7]
            q[8]=self.q1[8]
            q[9]=self.q1[9]
            q[10]=self.q1[10]
            robotController = self.sim.controller(0)
            qdes = q
            (qmin,qmax) = robot.getJointLimits()
            for i in xrange(len(qdes)-1):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.addLinear(qdes,1.1)

            # 7. Open the grippers
            step=2.1
            self.q1[7]=step
            self.q1[8]=-step
            self.q1[9]=-step
            self.q1[10]=step
            q[7]=self.q1[7]
            q[8]=self.q1[8]
            q[9]=self.q1[9]
            q[10]=self.q1[10]
            robotController = self.sim.controller(0)
            qdes = q
            (qmin,qmax) = robot.getJointLimits()
            for i in xrange(len(qdes)-1):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.addLinear(qdes,1.1)

            # 8. Back to initial position
            robotController = self.sim.controller(0)
            qdes = qinit
            (qmin,qmax) = robot.getJointLimits()
            for i in xrange(len(qdes)-1):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.addLinear(qinit,1.1)

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