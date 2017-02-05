import logging

from math import pi

from klampt import WorldModel
from klampt.vis import GLRealtimeProgram

from pensive.core import Store
from pensive.client import PensiveClient

logger = logging.getLogger(__name__)

class WorldViewer(GLRealtimeProgram):
    def __init__(self, path):
        self.world = WorldModel()
        if not self.world.readFile(path):
            raise RuntimeError('unable to load: {}'.format(path))
        GLRealtimeProgram.__init__(self, 'World Viewer')

        self.fps = 10.0
        self.dt = 1 / self.fps

        self.store = PensiveClient().default()
        self.db = None

    def display(self):
        self.world.drawGL()

    def idle(self):
        self.sync()

        robot = self.world.robot(0)
        q = self.db.get('/robot/current_config')
        if q:
            q = [x/180*3.141592 for x in q]
            robot.setConfig(q + [ 0 ])

        base_pose = self.db.get('/robot/base_pose')
        if base_pose:
            robot.link(0).setTransform(base_pose)

    def sync(self):
        self.db = Store()

        try:
            # query the database
            result = self.store.multi_get([
                '/robot/current_config',
                '/robot/base_pose',
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

if __name__ == "__main__":
    import sys
    import os

    viewer = WorldViewer('data/vis_world.xml')
    viewer.run()
