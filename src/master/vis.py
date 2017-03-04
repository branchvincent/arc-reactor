import logging

import numpy

from math import pi

from OpenGL.arrays import vbo
from OpenGL.GL import glEnable, glDisable, glPushMatrix, glPopMatrix, glMultMatrixf, glMatrixMode
from OpenGL.GL import glEnableClientState, glDisableClientState, glVertexPointerf, glColorPointerf, glDrawArrays, glPointSize
from OpenGL.GL import GL_MODELVIEW, GL_LIGHTING, GL_VERTEX_ARRAY, GL_COLOR_ARRAY, GL_POINTS

from klampt import WorldModel
from klampt.vis import GLRealtimeProgram
from klampt.vis.qtbackend import QtGLWindow

from .sync import AsyncUpdateMixin
from .world import update_world

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PointCloud(object):
    '''
    Helper class for using drawing point clouds with Vertex Buffer Objects.
    '''

    def __init__(self):
        self._xyz_vbo = None
        self._rgb_vbo = None

        self._pose = numpy.eye(4, 4)

    def update(self, xyz=None, rgb=None, pose=None):
        '''
        Update the point clouds points (XYZ or RGB) and/or pose.

        `xyz` and `rgb` are (N, 3) arrays of positions and non-normalized colors (0-255).
        `pose` is a 4x4 transformation matrix.

        Alternatively, `xyz` is an (N, 6) array of packed positions and colors.
        The first 3 elements of each vector are interpreted as position whereas
        hte second 3 elements of each vector are interpreted as non-normalized color (0-255).
        '''
        if pose is not None:
            self._pose = numpy.array(pose, dtype=numpy.float32)

        if xyz is not None:
            if not isinstance(xyz, numpy.ndarray):
                xyz = numpy.array(xyz)

            xyz = xyz.reshape((-1, xyz.shape[-1]))
            if xyz.shape[-1] == 6:
                # split up the position and color components
                rgb = xyz[:, 3:6]
                xyz = xyz[:, 0:3]
            if xyz.shape[-1] != 3:
                raise RuntimeError('invalid point cloud XYZ dimension: {}'.format(xyz.shape))

            # ref: http://pyopengl.sourceforge.net/context/tutorials/shader_1.html
            self._xyz_vbo = vbo.VBO(xyz.astype(numpy.float32))
            logger.debug('loaded point cloud with {} XYZ points'.format(xyz.shape))

        if rgb is not None:
            if not isinstance(xyz, numpy.ndarray):
                rgb = numpy.array(rgb)

            rgb = rgb.reshape((-1, rgb.shape[-1]))
            if rgb.shape[-1] != 3:
                raise RuntimeError('invalid point cloud RGB dimension: {}'.format(rgb.shape))

            self._rgb_vbo = vbo.VBO(rgb.astype(numpy.float32) / 255)
            logger.debug('loaded point cloud with {} RGB points'.format(rgb.shape))

    def draw(self):
        '''
        Draw the point cloud.
        '''
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glMultMatrixf(self._pose.T)

        glDisable(GL_LIGHTING)
        glPointSize(2)

        if self._xyz_vbo:
            # ref: http://stackoverflow.com/questions/16558819/vertex-buffer-objects-in-pyopengl-vertex-index-and-colour
            with self._xyz_vbo:
                glEnableClientState(GL_VERTEX_ARRAY)
                glVertexPointerf(self._xyz_vbo)

                if self._rgb_vbo:
                    # check for dimension match to avoid segmentation faults
                    if len(self._rgb_vbo) != len(self._xyz_vbo):
                        raise RuntimeError('point cloud XYZ and RGB length mismatch: {} vs {}'.format(len(self._xyz_vbo), len(self._rgb_vbo)))

                    with self._rgb_vbo:
                        # add color
                        glEnableClientState(GL_COLOR_ARRAY)
                        glColorPointerf(self._rgb_vbo)
                        glDrawArrays(GL_POINTS, 0, len(self._xyz_vbo))
                else:
                    # draw without color
                    glDrawArrays(GL_POINTS, 0, len(self._xyz_vbo))

        glDisableClientState(GL_COLOR_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)

        glEnable(GL_LIGHTING)

        glPopMatrix()

class WorldViewer(GLRealtimeProgram):
    def __init__(self):
        GLRealtimeProgram.__init__(self, 'World Viewer')

        self.world = WorldModel()

        self.fps = 10.0
        self.dt = 1 / self.fps

        self.pre_drawables = []
        self.post_drawables = []

    def display(self):
        for drawable in self.pre_drawables:
            drawable.draw()

        self.world.drawGL()

        for drawable in self.post_drawables:
            drawable.draw()

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
            (3, '/camera/camera1/timestamp'),
            (30, '/camera/camera1/point_cloud'),
            (30, '/camera/camera1/aligned_image'),
        ]

        self.timestamps = {}

        self.point_clouds = {}

    def update(self):
        update_world(self.db, self.program.world, self.timestamps)

        # update camera point clouds
        for name in ['camera1']:
            self._update_camera(name)

    def _update_camera(self, name):
        cloud_name = '{}_pc'.format(name)

        stamp = self.db.get('/camera/{}/timestamp'.format(name))
        if stamp <= self.timestamps.get(cloud_name, 0):
            # point cloud is up to date
            return

        self.timestamps[cloud_name] = stamp

        logger.info('updating {} point cloud'.format(name))

        cloud_xyz = self.db.get('/camera/{}/point_cloud'.format(name))
        cloud_rgb = self.db.get('/camera/{}/aligned_image'.format(name))

        cloud = self.point_clouds.get(cloud_name)
        if not cloud:
            # construct and register a new point cloud
            cloud = PointCloud()
            self.program.post_drawables.append(cloud)
            self.point_clouds[cloud_name] = cloud

        # perform the update
        mask = cloud_xyz[:,:,2] > 0
        cloud.update(xyz=cloud_xyz[mask], rgb=cloud_rgb[mask], pose=self.db.get('/camera/{}/pose'.format(name)))

if __name__ == '__main__':
    from PyQt4.QtGui import QApplication
    app = QApplication([])
    app.setApplicationName('ARC Reactor')

    window = WorldViewerWindow()
    window.show()

    from .sync import exec_async
    exec_async(app, [window], db_period=33)
