#!/usr/bin/python

import sys
from klampt import *
from klampt.vis.glrobotprogram import *
from klampt.model import ik,coordinates
from klampt.math import so3
import random
import importlib
import os
import time
import sys
#FOR DEFAULT JOINT-BY-JOINT KEYMAP: set keymap=None
keymap = None

#FOR CUSTOM KEYMAPS: set up keymap to define how keys map to velocities.
#keymap is a map from key name to (robot index,velocity vector) pairs.
#Key names can either be single keys or names of special keys
#'left','up','down','right', 'home', 'insert', 'end', and the function keys 'f1',...,'f12'.
#keymap = {'up':(0,[0,1]),'down':(0,[0,-1]),'left':(0,[-1,0]),'right':(0,[1,0])}

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



class MyGLViewer(GLSimulationProgram):
    def __init__(self,world):
        global keymap
        GLSimulationProgram.__init__(self,world,"My GL program")
        self.world = world
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
        #Put your initialization code here

    def control_loop(self):
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*self.world.robot(r).numLinks() for r in range(self.world.numRobots())]
        
        # print rvels
        # print self.world.robot(0).link(7).getWorldPosition([0,0,0])
        #send to the robot(s)
        # self.target=vectorops.add(self.world.rigidObject(self.world.numRigidObjects()-1).getTransform()[1],[-0.05,0,0.1])
        robot=self.world.robot(0)
        ee_link=6
        ee_local_p1=[0,0,0]
        ee_local_p2=[0.01,0,0]
        ee_world_p1=self.target
        ee_world_p2=vectorops.add(self.target,[0,0,-0.01])
        # goal = ik.objective(robot.link(ee_link),local=ee_local_p1,world=ee_world_p1)
        goal = ik.objective(robot.link(ee_link),local=[ee_local_p1,ee_local_p2],world=[ee_world_p1,ee_world_p2])
        old_T=robot.link(ee_link).getTransform()
        old_R,old_t=old_T
        s=ik.solver(goal)
        s.setMaxIters(100)
        s.setTolerance(0.001)
        s.solve()
        q= robot.getConfig()
        robotController = self.sim.controller(0)
        qdes = q
        (qmin,qmax) = robot.getJointLimits()
        for i in xrange(len(qdes)):
            qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
        robotController.setPIDCommand(qdes,rvels[0])
        obj = self.sim.world.rigidObject(self.world.numRigidObjects()-1)
        #get a SimBody object
        body = self.sim.body(obj)

        if self.activeObjects:
            t1=robot.link(ee_link).getWorldPosition([0,0,0.1])
            t2=body.getTransform()[1]
            t=vectorops.sub(t1,t2)
            n=vectorops.norm(t)

            body.applyForceAtPoint(vectorops.mul(vectorops.div(t,n),min(5/n,40)),t1)


            # T=body.getTransform()
            # R,t=T           
            # body.enableDynamics(False)
            # if self.flag==0:
            #     self.flag=1
            #     self.t=robot.link(ee_link).getLocalPosition(t)

            # new_T=robot.link(ee_link).getTransform()
            # new_R,new_t=new_T
            # change_R=so3.mul(new_R,so3.inv(old_R))
            
            # R=so3.mul(change_R,R)
            # body.setTransform(R,robot.link(ee_link).getWorldPosition(self.t))
        else:
            body.enableDynamics(True)
            self.flag=0
        return

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
        if c in self.keymap:
            self.current_velocities[c]=self.keymap[c]
        else:
            # print self.target
            step=0.05
            if c == 'u':
                self.target[0]+=step
            elif c== 'j':
                self.target[0]-=step
            elif c=='i':
                self.target[1]+=step
            elif c == 'k':
                self.target[1]-=step
            elif c== 'o':
                self.target[2]+=step
            elif c== 'l':   
                self.target[2]-=step
            elif c=='v':
                self.activeObjects= not self.activeObjects
            elif c=='p':
                print 'target:',self.target

            else:
                GLSimulationProgram.keyboardfunc(self,c,x,y)
        self.refresh()

    def keyboardupfunc(self,c,x,y):
        if c in self.current_velocities:
            del self.current_velocities[c]
        return


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
