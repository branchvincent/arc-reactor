import logging

from time import time, sleep

from math import pi

import numpy
from matplotlib import cm

from OpenGL.arrays import vbo
from OpenGL.GL import glEnable, glDisable
from OpenGL.GL import glEnableClientState, glDisableClientState, glVertexPointerf, glDrawArrays, glColor, glLineWidth
from OpenGL.GL import GL_LIGHTING, GL_VERTEX_ARRAY, GL_LINE_STRIP, GL_STATIC_DRAW

from klampt.vis import GLRealtimeProgram

from PyQt4.QtGui import QMainWindow

from master.world import build_world

from .ui.motion_plan import Ui_MotionPlanWindow

logger = logging.getLogger(__name__)

class WorldViewer(GLRealtimeProgram):
    def __init__(self, store, update_callback=None):
        GLRealtimeProgram.__init__(self, 'Motion Plan Checkpoint')

        self.world = build_world(store)
        self.robot = self.world.robot('tx90l')

        self.robot_path = []

        mp = store.get('/robot/waypoints')

        self.dts = [wp[0] for wp in mp]
        self.commands = [{'robot': self.robot.getConfig()}] + [wp[1] for wp in mp]

        self.fps = 30.0
        self.dt = 1 / self.fps

        self.speed_scale = 1
        self.end_delay = 1

        self.n = 0
        self.start_time = -self.end_delay
        self.duration = sum(self.dts)

        self.dofs = self.robot.numLinks()

        self.trace_vbos = []
        for i in range(1, self.dofs):
            trace = self.trace_link(self.commands, i)
            data = numpy.array(trace, dtype=numpy.float32)
            color = cm.gist_rainbow(float(i - 1) / (self.dofs - 2))
            self.trace_vbos.append((color, vbo.VBO(data, GL_STATIC_DRAW)))

        self.pause = False
        self.update_callback = update_callback

    def trace_link(self, plan, link):
        trace = []

        for cmd in plan:
            self.robot.setConfig(cmd['robot'])
            trace.append(self.robot.link(link).getTransform()[1])

        return trace

    def query_time(self, t, hint=None):
        if t < 0:
            return self.commands[0]

        for i in range(len(self.dts)):
            dt = self.dts[i]
            if t > dt:
                t -= dt
                continue

            ratio = t / dt
            (cmd_start, cmd_end) = self.commands[i:i+2]

            command = {}

            # first-order hold for continuous variables
            for name in ['robot', 'shelf', 'gripper']:
                if name in cmd_start and name in cmd_end:
                    command[name] = [(1 - ratio)*x + ratio*y for (x, y) in zip(cmd_start[name], cmd_end[name])]

            # zero-order hold for discrete variables
            for name in ['vacuum']:
                if name in cmd_start:
                    command[name] = cmd_start[name]

            return command

        return self.commands[-1]

    def display(self):
        self.world.drawGL()

        glDisable(GL_LIGHTING)
        glLineWidth(2.0)

        glEnableClientState(GL_VERTEX_ARRAY)

        for (color, trace) in self.trace_vbos:
            glColor(*color)
            with trace:
                glVertexPointerf(trace)
                glDrawArrays(GL_LINE_STRIP, 0, len(trace))

        glDisableClientState(GL_VERTEX_ARRAY)

        glEnable(GL_LIGHTING)

    def idle(self):
        if not self.commands:
            return

        if not self.pause:
            self.current_time = self.speed_scale * (time() - self.start_time)
            if self.current_time > self.duration + self.end_delay:
                self.start_time = time() + self.end_delay

            if self.update_callback:
                self.update_callback(min([max([0, self.current_time]), self.duration]), self.duration)

        command = self.query_time(self.current_time)

        if 'robot' in command:
            self.world.robot('tx90l').setConfig(command['robot'])
        if 'shelf' in command:
            self.world.robot('shelf').setConfig(command['shelf'])

    def mousefunc(self,button,state,x,y):
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

class WorldViewerWindow(QMainWindow):
    def __init__(self, store):
        super(WorldViewerWindow, self).__init__()

        self.ui = Ui_MotionPlanWindow()
        self.ui.setupUi(self)

        self.program = WorldViewer(store, self.update)
        self.ui.view.setProgram(self.program)
        self.setWindowTitle(self.program.name)
        self.ui.view.setMaximumSize(1920, 1080)

        self.ui.run_button.clicked.connect(self.toggle_pause)
        self.ui.time_slider.sliderMoved.connect(self.change_slider)

        self.ui.approve_button.setEnabled(False)
        self.ui.reject_button.setEnabled(False)

    def toggle_pause(self):
        self.program.pause = not self.program.pause
        self.ui.time_slider.setEnabled(self.program.pause)

        if not self.program.pause:
            self.program.start_time = time() - self.program.current_time / self.program.speed_scale
            self.ui.run_button.setText('Pause')
        else:
            self.ui.run_button.setText('Play')

    def change_slider(self, value):
        if self.program.pause:
            self.program.current_time = value / 1000.0

        self.sync_label()

    def update(self, time, duration):
        self.ui.time_slider.setMaximum(1000 * duration)
        self.ui.time_slider.setValue(1000 * time)

        self.sync_label()

    def sync_label(self):
        time = self.ui.time_slider.value() / 1000.0
        duration = self.ui.time_slider.maximum() / 1000.0
        self.ui.time_label.setText('{:.2f}s of {:.2f}s'.format(time, duration))

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
