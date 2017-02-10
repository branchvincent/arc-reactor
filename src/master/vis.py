import logging

import numpy

from math import pi

from klampt import WorldModel
from klampt.vis import GLRealtimeProgram

from pensive.core import Store
from pensive.client import PensiveClient

from .world import update_world

logger = logging.getLogger(__name__)

class WorldViewer(GLRealtimeProgram):
    def __init__(self):
        GLRealtimeProgram.__init__(self, 'World Viewer')

        self.world = WorldModel()

        self.fps = 10.0
        self.dt = 1 / self.fps

        self.store = PensiveClient().default()
        self.db = None

    def display(self):
        self.world.drawGL()

    def idle(self):
        self.sync()
        update_world(self.store, self.world)

    def sync(self):
        self.db = Store()

        try:
            # query the database
            result = self.store.multi_get([
                '/robot',
                '/shelf'
            ])
        except:
            logger.exception('UI update query failed')
        else:
            # build a local store of just the queried items so that
            # the UI code can use the nice `Store` interfaces
            for (key, value) in result.iteritems():
                self.db.put(key, value)

    def mousefunc(self,button,state,x,y):
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

if __name__ == '__main__':
    import sys
    import os

    WorldViewer().run()
