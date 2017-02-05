import logging

import numpy

from math import pi

from klampt import WorldModel
from klampt.vis import GLRealtimeProgram

from pensive.core import Store
from pensive.client import PensiveClient

logger = logging.getLogger(__name__)

def _numpy2klampt(T):
    # remove singleton dimensions
    T = T.squeeze()
    # check output based on shape
    if T.shape in [(4, 4), (3, 4)]:
        # convert to SE3
        return (list(T[:3, :3].flat), list(T[:3, 3].flat))
    elif T.shape == (3, 3):
        return list(T.flat)
    else:
        raise RuntimeError('unknown array shape for conversion: {}'.format(T.shape))

def _deg2rad(x):
    return numpy.array(x) * pi / 180.0

def _rad2deg(x):
    return numpy.array(x) * 180.0 / pi

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

        # robot arm
        tx90l = self.world.robot('tx90l')

        base_pose = self.db.get('/robot/base_pose')
        if base_pose is not None:
            tx90l.link(0).setParentTransform(*_numpy2klampt(base_pose))

        q = self.db.get('/robot/current_config')
        if q is not None:
            tx90l.setConfig(_deg2rad(q) + [ 0 ])
        else:
            # force forward kinematics update
            tx90l.setConfig(tx90l.getConfig())

        # shelf
        shelf = self.world.robot('shelf')

        shelf_pose = self.db.get('/shelf/pose')
        if shelf_pose is not None:
            shelf.link(0).setParentTransform(*_numpy2klampt(shelf_pose))

        q = self.db.get('/shelf/current_angle')
        if q is not None:
            shelf.setConfig([0, q])
        else:
            # force forward kinematics update
            shelf.setConfig(shelf.getConfig())

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

    viewer = WorldViewer('data/vis_world.xml')
    viewer.run()
