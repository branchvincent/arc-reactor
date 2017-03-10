import logging

import numpy
from matplotlib import cm

from time import time, sleep

from math import pi

from OpenGL.arrays import vbo
from OpenGL.GL import glEnable, glDisable
from OpenGL.GL import glEnableClientState, glDisableClientState, glVertexPointerf, glColorPointerf, glDrawArrays, glColor, glLineWidth
from OpenGL.GL import GL_LIGHTING, GL_VERTEX_ARRAY, GL_COLOR_ARRAY, GL_LINE_STRIP, GL_STATIC_DRAW

from klampt import WorldModel
from klampt.vis import GLRealtimeProgram
from klampt.vis.qtbackend import QtGLWindow
from klampt.math import vectorops, so3
numpy
from master.world import build_world, update_world

logger = logging.getLogger(__name__)

class WorldViewer(GLRealtimeProgram):
    def __init__(self, store):
        GLRealtimeProgram.__init__(self, 'Motion Plan Checkpoint')

        self.world = build_world(store)
        self.robot = self.world.robot('tx90l')

        self.robot_path = []
        self.motion_plan = store.get('/robot/waypoints')

        t = 0
        for waypoint in self.motion_plan:
            dt = waypoint[0]
            waypoint[0] = t
            t += dt

        self.fps = 30.0
        self.dt = 1 / self.fps

        self.speed_scale = 1
        self.end_delay = 1

        self.n = 0
        self.start_time = -self.end_delay
        self.waypoint_index = None

        self.dofs = self.robot.numLinks()

        self.trace_vbos = []
        for i in range(1, self.dofs):
            trace = self.trace_link(self.motion_plan, i)
            data = numpy.array(trace, dtype=numpy.float32)
            color = cm.gist_rainbow(float(i - 1) / (self.dofs - 2))
            self.trace_vbos.append((color, vbo.VBO(data, GL_STATIC_DRAW)))

    def trace_link(self, plan, link):
        trace = []

        for (t, cmd) in plan:
            self.robot.setConfig(cmd['robot'])
            trace.append(self.robot.link(link).getTransform()[1])

        return trace

    # def resample(self, max_dt=None, max_dq=None, max_dx=None):
    #     if max_dq is not None:
    #         try:
    #             if len(max_dq) < self.dofs:
    #                 raise TypeError
    #         except TypeError:
    #             max_dq = [max_dq] * self.dofs

    #     if max_dx is not None:
    #         try:
    #             if len(max_dx) < self.dofs:
    #                 raise TypeError
    #         except TypeError:
    #             max_dx = [max_dx] * self.dofs

    #     trajectory = [self.motion_plan[0]]
    #     i = 0
    #     step = 1.0

    #     while i < len(self.motion_plan):
    #         # query motion plan
    #         (t, cmd) = self.query_index(i + step)

    #         if max_dt is not None:
    #             # check time differences
    #             if t - trajectory[-1][0] > max_dt:
    #                 step /= 2
    #                 continue

    #         if max_dq is not None:
    #             # check joint differences
    #             dq = [a - b for (a, b) in zip(cmd['robot'], trajectory[-1][1]['robot'])]
    #             if any([dq > lim for (dq, lim) in zip(dq, max_dq)]):
    #                 step /= 2
    #                 continue

    #         if max_dx is not None:
    #             for i in range(self.dofs):
    #                 self.robot.setConfig(trajectory[-1][1]['robot'])
    #                 (Ra, ta) = self.robot.link(i).getTransform()

    #                 self.robot.setConfig(cmd['robot'])
    #                 (Rb, tb) = self.robot.link(i).getTransform()

    #                 # check angle
    #                 if so3.distance(Ra, Rb) > max_dq[i][0]:
    #                     step /= 2
    #                     continue

    #                 # check distance
    #                 if vectorops.distance(ta, tb) > max_dq[i][1]:
    #                     step /= 2
    #                     continue

    #         trajectory.append((t, cmd))
    #         i += step
    #         step = 1.0

    #     return trajectory

    # def query_index(self, i):
    #     command = {}

    #     if i <= 0:
    #         # before start
    #         return self.motion_plan[0]
    #     elif i >= len(self.motion_plan) - 1:
    #         # after end
    #         return self.motion_plan[-1]
    #     else:
    #         segment = self.motion_plan[int(i):int(i)+2]

    #         # first-order hold for continuous variables
    #         ratio = i % 1
    #         for name in ['robot', 'shelf', 'gripper']:
    #             if name in segment[0][1] and name in segment[1][1]:
    #                 command[name] = [ratio*a + (1 - ratio)*b for (a, b) in zip(segment[0][1][name], segment[1][1][name])]

    #         # zero-order hold for discrete variables
    #         for name in ['vacuum']:
    #             if name in segment[0][1]:
    #                 command[name] = segment[0][1][name]

    #         return ((1 - ratio)*segment[0][0] + ratio*segment[1][0], command)

    def query_time(self, t, hint=None):
        command = {}

        if t < 0:
            # before start
            command = self.motion_plan[0][1]
        elif t > self.motion_plan[-1][0]:
            # after end
            command = self.motion_plan[-1][1]
        else:
            # find the waypoint before and after the current time
            segment = None
            for i in range(hint or 1, len(self.motion_plan)):
                if self.motion_plan[i][0] < t:
                    continue

                segment = self.motion_plan[i-1:i+1]
                break

            # first-order hold for continuous variables
            ratio = (segment[0][0] - t) / (segment[1][0] - segment[0][0])
            for name in ['robot', 'shelf', 'gripper']:
                if name in segment[0][1] and name in segment[1][1]:
                    command[name] = [ratio*a + (1 - ratio)*b for (a, b) in zip(segment[0][1][name], segment[1][1][name])]

            # zero-order hold for discrete variables
            for name in ['vacuum']:
                if name in segment[0][1]:
                    command[name] = segment[0][1][name]

        return command

    def display(self):
        self.world.drawGL()

        glDisable(GL_LIGHTING)
        glLineWidth(2.0)

        glEnableClientState(GL_VERTEX_ARRAY)

        for (color, vbo) in self.trace_vbos:
            glColor(*color)
            with vbo:
                glVertexPointerf(vbo)
                glDrawArrays(GL_LINE_STRIP, 0, len(vbo))

        glDisableClientState(GL_VERTEX_ARRAY)

        glEnable(GL_LIGHTING)

    def idle(self):
        if not self.motion_plan:
            return

        current_time = self.speed_scale * (time() - self.start_time)
        if current_time > self.motion_plan[-1][0] + self.end_delay:
            self.start_time = time() + self.end_delay

        command = self.query_time(current_time)

        if 'robot' in command:
            self.world.robot('tx90l').setConfig(command['robot'])
        if 'shelf' in command:
            self.world.robot('shelf').setConfig(command['shelf'])

    def mousefunc(self,button,state,x,y):
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

class WorldViewerWindow(QtGLWindow):
    def __init__(self, store):
        super(WorldViewerWindow, self).__init__()

        self.setProgram(WorldViewer(store))
        self.setWindowTitle(self.program.name)
        self.setMaximumSize(1920, 1080)

def run(modal=True, store=None):
    from pensive.client import PensiveClient
    store = store or PensiveClient().default()

    from PyQt4.QtGui import QApplication
    app = QApplication.instance()
    if app:
        embedded = True
    else:
        embedded = False
        app = QApplication([])
        app.setApplicationName('ARC Reactor')

    window = WorldViewerWindow(store)
    if modal:
        from PyQt4.QtCore import Qt
        window.setWindowModality(Qt.ApplicationModal)
    window.show()

    if not embedded:
        app.exec_()

if __name__ == '__main__':
    run()
