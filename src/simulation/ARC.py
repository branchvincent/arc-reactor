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
        self.current_velocities = {}
        self.target=0
        # self.target=vectorops.add(self.world.rigidObject(self.world.numRigidObjects()-1).getTransform()[1],[-0.05,0,0.1])
        self.numberOfObjects=world.numRigidObjects()
        self.graspedObjects=[False]*self.numberOfObjects
        self.start_flag=0
        self.flag=0
        self.t=[]
        self.state='idle'
        self.last_state_end=self.sim.getTime()
        self.start_T=[]
        self.end_T=[]
        self.object_p=[]
        self.retract_T=[]
        self.idle_T=[]
        self.score=0
        self.waiting_list=range(self.numberOfObjects)
        self.target_in_box=[]
        self.globals=Globals(world)
        self.cspace=TestCSpace(self.globals)
        self.idleT=[]
        self.output=[]
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
        ee_link=6
        ee_local_p1=[-0.015,-0.02,0.27]
        q_curr=robot.getConfig()
        q_curr[3]=-q_curr[3]
        q_curr[5]=-q_curr[5]
        q_output=vectorops.mul(q_curr[1:],angle_to_degree)

        milestone=(0.1, {
              'robot': q_output,
              'gripper': [0,0,0],
              'vaccum': [0]
            })
        self.output.append(milestone)
        if self.start_flag==0:
            curr_position=robot.link(ee_link).getWorldPosition(ee_local)
            curr_orientation,p=robot.link(ee_link).getTransform()
            self.start_T=[curr_orientation,curr_position]
            self.start_flag=1
        local_p=[]
        if self.state=='idle':
            t=0.5
            if self.score==2:
                print 'finished!'
                f=open('test.json','w')
                json.dump(self.output,f)
                f.close()
                exit()
            else:
                if self.sim.getTime()-self.last_state_end<t:
                    u=(self.sim.getTime()-self.last_state_end)/t
                    self.end_T=[[0,0,-1, 0,1,0, 1,0,0],idle_position]
                    # self.idle_T=self.end_T
                    # self.sim.controller(0).setPIDCommand(self.idleq,[0]*7)          
                    [local_p,world_p]=interpolate(self.start_T,self.end_T,u,1)
                    # constraints=ik.objective(robot.link(ee_link),local=local_p,world=world_p)
                    # traj1 = cartesian_trajectory.cartesian_interpolate_linear(robot,self.start_T,self.end_T,constraints,delta=1e-2,maximize=False)
                    # print 'traj1:',traj1[0]
                else:
                    # print 'idle', robot.getConfig()
                    self.set_state('find_target')
        elif self.state=='find_target':
            
    
            h=0
            for i in self.waiting_list:
                if self.sim.world.rigidObject(i).getTransform()[1][2]>h:
                    self.target=i
                    h=self.sim.world.rigidObject(i).getTransform()[1][2]
            print 'target is ', self.target
            # self.target=0
            # self.object_p=self.sim.world.rigidObject(self.target).getTransform()[1]
            bb1,bb2=self.sim.world.rigidObject(self.target).geometry().getBB()
            self.object_p=vectorops.div(vectorops.add(bb1,bb2),2)
            # print 'object xform', self.sim.world.rigidObject(self.target).getTransform()[1]
            # print 'object_p',self.object_p
            h=self.object_p[2]+box_vacuum_offset[2]
            if self.object_p[1]>0:
                end_p=approach_p1
                end_p[2]=h
            else:
                end_p=approach_p2
                end_p[2]=h
            d=vectorops.distance(end_p[:1],self.object_p[:1])
            dx=self.object_p[0]-end_p[0]
            dy=self.object_p[1]-end_p[1]
            self.end_T=[[0,0,-1, -dy/d,dx/d,0, dx/d,dy/d,0],end_p]
            print 'approach start position',end_p
            self.retract_T=[[0,0,-1, -dy/d,dx/d,0, dx/d,dy/d,0],end_p]
            print 'retract_T',self.retract_T
            self.set_state('preposition')
        elif self.state=='preposition':
            t=0.5
            if self.sim.getTime()-self.last_state_end<t:
                u=(self.sim.getTime()-self.last_state_end)/t
                [local_p,world_p]=interpolate(self.start_T,self.end_T,u,1)
            else:
                self.end_T[1]=vectorops.add(self.object_p,[0.10,0,0.12])
                self.set_state('pregrasp')
        elif self.state=='pregrasp':
            t=0.5
            # print 'pregrasp'
            if self.sim.getTime()-self.last_state_end<t:
                u=(self.sim.getTime()-self.last_state_end)/t
                [local_p,world_p]=interpolate(self.start_T,self.end_T,u,1)
            else:
                self.end_T[1][2]-=0.05
                self.set_state('grasp')
        elif self.state=='grasp':
            
            # print 'grasp'
            t=0.5
            if self.sim.getTime()-self.last_state_end<t:
                u=(self.sim.getTime()-self.last_state_end)/t
                [local_p,world_p]=interpolate(self.start_T,self.end_T,u,1)
            else:
                self.end_T[1][2]+=0.05
                self.set_state('prograsp')
        elif self.state=='prograsp':
            # print 'grasp'
            self.graspedObjects[self.target]=True
            t=0.5
            if self.sim.getTime()-self.last_state_end<t:
                u=(self.sim.getTime()-self.last_state_end)/t
                [local_p,world_p]=interpolate(self.start_T,self.end_T,u,1)
            else:
                self.end_T=self.retract_T
                self.end_T[1][2]+=0.01
                self.set_state('retract')
        elif self.state=='retract':
            # print 'retract'
            t=0.5
            if self.sim.getTime()-self.last_state_end<t:
                u=(self.sim.getTime()-self.last_state_end)/t
                [local_p,world_p]=interpolate(self.start_T,self.end_T,u,1)
            else:
                self.set_state('go_to_tote')
        elif self.state=='go_to_tote':
            x=robot.link(ee_link).getTransform()[1][0]
            if x>(order_box_min[0]+order_box_max[0])/2:
                q=robot.getConfig()
                q[1]+=0.02
                self.sim.controller(0).setPIDCommand(q,[0]*7)
                # self.output.append(vectorops.mul(q[1:],angle_to_degree))
            else:
                bb1,bb2=self.sim.world.rigidObject(self.target).geometry().getBB()
                bbx=bb2[0]-bb1[0]
                bby=bb2[1]-bb1[1]
                print 'BB:',bbx,bby
                # self.end_T[1]=[order_box_min[0]+bbx/2-0.01,order_box_min[1]+bby/2+0.21-self.target*0.1,0.8]
                self.end_T[1]=self.find_placement()
                # if self.score>0:
                #     pass
                self.end_T[0]=[0,0,-1,-1,0,0,0,1,0]
                self.set_state('place')
        elif self.state=='place':
            t=0.5
            if self.sim.getTime()-self.last_state_end<t:
                u=(self.sim.getTime()-self.last_state_end)/t
                [local_p,world_p]=interpolate(self.start_T,self.end_T,u,0)
            else:
                self.end_T[1][2]-=0.01
                self.set_state('release')
        elif self.state=='release':
            t=0.5
            if self.sim.getTime()-self.last_state_end<t:
                u=(self.sim.getTime()-self.last_state_end)/t
                [local_p,world_p]=interpolate(self.start_T,self.end_T,u,0)
            else:
                self.graspedObjects[self.target]=False
                self.check_target()
                self.set_state('idle')



        old_T=robot.link(ee_link).getTransform()
        old_R,old_t=old_T
        if local_p:
            q_old=robot.getConfig()
            goal = ik.objective(robot.link(ee_link),local=local_p,world=world_p)
            # s=ik.solver(goal)
            # s.setMaxIters(100)
            # s.setTolerance(0.2)
            # if s.solve():
            #     pass
            # else:
            #     print "IK failed"
            # while not s.solve():
            #     q_restart=[0,0,0,0,0,0,0]
            #     q=robot.getConfig()
            #     (qmin,qmax) = robot.getJointLimits()
            #     for i in range(7):
            #         q_restart[i]=random.uniform(-1,1)+q[i]
            #         q_restart[i] = min(qmax[i],max(q_restart[i],qmin[i]))
            #     print q_restart
            #     robot.setConfig(q_restart)
            #     if s.solve():
            #         print "IK solved"
            #     else:
            #         print "IK failed"
            s=ik.solve_global(goal)
            q= robot.getConfig()
            if not feasible(robot,q,q_old):
                s=ik.solve_global(goal)
                q=robot.getConfig()

            robotController = self.sim.controller(0)
            qdes = q
            (qmin,qmax) = robot.getJointLimits()
            for i in xrange(len(qdes)):
                qdes[i] = min(qmax[i],max(qdes[i],qmin[i]))
            robotController.setPIDCommand(qdes,rvels[0])
            # self.output.append(vectorops.mul(qdes[1:],angle_to_degree))

        obj = self.sim.world.rigidObject(self.target)
        #get a SimBody object
        body = self.sim.body(obj)
        if self.graspedObjects[self.target]:
                T=body.getTransform()
                R,t=T           
                body.enableDynamics(False)
                if self.flag==0:
                    self.flag=1
                    self.t=robot.link(ee_link).getLocalPosition(t)
                # print 'here'
                new_T=robot.link(ee_link).getTransform()
                new_R,new_t=new_T
                change_R=so3.mul(new_R,so3.inv(old_R))
                
                R=so3.mul(change_R,R)
                body.setTransform(R,robot.link(ee_link).getWorldPosition(self.t))
        else:
            body.enableDynamics(True)
            self.flag=0
        return


    def check_target(self):
        p=self.sim.world.rigidObject(self.target).getTransform()[1]
        if p[2]>0:
            self.target_in_box.append(self.target)
            self.waiting_list.remove(self.target)
            self.score+=1

    def find_placement(self):
        p=[]
        radius=0.07
        for i in self.target_in_box:
            p.append(self.sim.world.rigidObject(i).getTransform()[1])
        flag=0
        xmin=order_box_min[0]+radius+0.1
        xmax=order_box_max[0]-radius
        ymin=order_box_min[1]+radius+0.05
        ymax=order_box_max[1]-radius+0.05
        count=0
        while not flag:
            x=random.uniform(xmin,xmax)
            y=random.uniform(ymin,ymax)
            flag=1
            if p:
                for i in range(len(p)):
                    if vectorops.distance([x,y],[p[i][0],p[i][1]])>2*radius:
                        pass
                    else:
                        flag=0
                        count+=1
            if count>4:
                return [(xmin+xmax)/2,(ymin+ymax)/2,0.5]
        t=[x,y,0.5]
        return t




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
