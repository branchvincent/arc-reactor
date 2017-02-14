import logging

import numpy

from time import time, sleep

from math import pi

from klampt import WorldModel
from klampt.vis import GLRealtimeProgram

from pensive.core import Store
from pensive.client import PensiveClient

from master.world import build_world, update_world

logger = logging.getLogger(__name__)

class WorldViewer(GLRealtimeProgram):
    def __init__(self):
        GLRealtimeProgram.__init__(self, 'Motion Plan Checkpoint')

        store = PensiveClient().default()
        self.world = build_world(store)

        self.robot_path = []
        self.motion_plan = store.get('/robot/waypoints')

        t = 0
        for waypoint in self.motion_plan:
            dt = waypoint[0]
            waypoint[0] = t
            t += dt

        self.fps = 30.0
        self.dt = 1 / self.fps

        self.speed_scale = 0.1
        self.end_delay = 1

        self.n = 0
        self.start_time = -self.end_delay
        self.waypoint_index = None

    def display(self):
        # hack to help Klampt not crash
        self.n += 1
        if self.n < 10:
            return

        self.world.drawGL()

    def idle(self):
        if not self.motion_plan:
            return

        current_time = self.speed_scale * (time() - self.start_time)
        if current_time > self.motion_plan[-1][0] + self.end_delay:
            self.start_time = time() + self.end_delay

        command = {}

        if current_time < 0:
            # before start
            command = self.motion_plan[0][1]
        elif current_time > self.motion_plan[-1][0]:
            # after end
            command = self.motion_plan[-1][1]
        else:
            # find the waypoint before and after the current time
            segment = None
            for i in range(1, len(self.motion_plan)):
                if self.motion_plan[i][0] < current_time:
                    continue

                segment = self.motion_plan[i-1:i+1]
                break

            # first-order hold for continuous variables
            ratio = (segment[0][0] - current_time) / (segment[1][0] - segment[0][0])
            for name in ['robot', 'shelf', 'gripper']:
                if name in segment[0][1] and name in segment[1][1]:
                    command[name] = [ratio*a + (1 - ratio)*b for (a, b) in zip(segment[0][1][name], segment[1][1][name])]

            # zero-order hold for discrete variables
            for name in ['vacuum']:
                if name in segment[0][1]:
                    command[name] = segment[0][1][name]

        if 'robot' in command:
            self.world.robot('tx90l').setConfig(command['robot'])
        if 'shelf' in command:
            self.world.robot('shelf').setConfig(command['shelf'])

    def mousefunc(self,button,state,x,y):
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

if __name__ == '__main__':
    import sys
    import os

    WorldViewer().run()