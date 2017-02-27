import logging

import numpy

from math import pi

from klampt import WorldModel
from klampt.vis import GLRealtimeProgram
from klampt.vis.qtbackend import QtGLWindow

from .sync import AsyncUpdateMixin
from .world import update_world

logger = logging.getLogger(__name__)

class WorldViewer(GLRealtimeProgram):
    def __init__(self):
        GLRealtimeProgram.__init__(self, 'World Viewer')

        self.world = WorldModel()

        self.fps = 10.0
        self.dt = 1 / self.fps

    def display(self):
        self.world.drawGL()    

    def mousefunc(self,button,state,x,y):
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

class WorldViewerWindow(QtGLWindow, AsyncUpdateMixin):
    def __init__(self):
        super(WorldViewerWindow, self).__init__()

        self.setProgram(WorldViewer())
        self.setWindowTitle('ARC Reactor Viewer')
        self.setMaximumSize(1920, 1080)
    
        self.setup_async()
        self.requests = [
            (3, '/robot'),
            (1, '/robot/current_config'),
            (3, '/shelf'),
            (3, '/item'),
            (3, '/box'),
            (3, '/camera/camera1/pose'),
        ]

        self.timestamps = {}

    def update(self):
        update_world(self.db, self.program.world, self.timestamps)

if __name__ == '__main__':
    from PyQt4.QtGui import QApplication
    app = QApplication([])
    app.setApplicationName('ARC Reactor')

    window = WorldViewerWindow()
    window.show()

    from .sync import exec_async
    exec_async(app, [window], db_period=33)
