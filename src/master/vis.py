import logging

import numpy
from scipy.interpolate import interp1d
from matplotlib import cm

from math import pi

from OpenGL.arrays import vbo
from OpenGL.GL import glEnable, glDisable, glPushMatrix, glPopMatrix, glMultMatrixf, glMatrixMode
from OpenGL.GL import glEnableClientState, glDisableClientState, glVertexPointerf, glColorPointerf, glDrawArrays, glPointSize
from OpenGL.GL import GL_MODELVIEW, GL_LIGHTING, GL_VERTEX_ARRAY, GL_COLOR_ARRAY, GL_POINTS

from klampt import WorldModel
from klampt.vis import GLRealtimeProgram, gldraw
from klampt.vis.qtbackend import QtGLWindow
from klampt.math import se3

from .sync import AsyncUpdateMixin
from .world import update_world

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class PointCloudRender(object):
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

        `xyz` and `rgb` are (N, 3) arrays of positions and colors. The normalization of colors
        is assumed from their datatype.  `pose` is a 4x4 transformation matrix.

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

            # infer normalization from datatype
            if numpy.issubdtype(rgb.dtype, numpy.integer):
                normalization = 255
            else:
                normalization = 1

            self._rgb_vbo = vbo.VBO(rgb.astype(numpy.float32) / normalization)
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

        poses = [se3.identity()]

        for i in range(self.world.numRobots()):
            robot = self.world.robot(i)
            poses.append(robot.link(0).getTransform())
            poses.append(robot.link(robot.numLinks()-1).getTransform())

        for i in range(self.world.numRigidObjects()):
            poses.append(self.world.rigidObject(i).getTransform())

        for pose in poses:
            gldraw.xform_widget(pose, 0.1, 0.01, fancy=True)

    def mousefunc(self,button,state,x,y):
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

class WorldViewerWindow(QtGLWindow, AsyncUpdateMixin):
    def __init__(self):
        super(WorldViewerWindow, self).__init__()

        self.setProgram(WorldViewer())
        self.setWindowTitle(self.program.name)
        self.setMaximumSize(1920, 1080)

        self.setup_async()
        self.requests = [
            (3, '/robot'),
            (1, '/robot/current_config'),
            (3, '/shelf'),
            (3, '/item'),
            (3, '/box'),
            (3, '/tote'),
            (3, '/camera/shelf0/pose'),
            (3, '/camera/shelf0/timestamp'),
            (3, '/camera/stow/pose'),
            (3, '/camera/stow/timestamp'),
        ]

        self.timestamps = {}

        self.point_clouds = {}

        self.options = {}

    def update(self):
        update_world(self.db, self.program.world, self.timestamps, ignore=['items'])

        # update camera point clouds
        for name in ['shelf0', 'stow']:
            self._update_camera(name)

        # update item point clouds
        for name in self.db.get('item'):
            self._update_item(name)

    def _update_item(self, name):
        cloud_name = '{}_pc'.format(name)

        # check if point cloud is modified
        stamp = self.db.get(['item', name, 'timestamp'])
        if stamp <= self.timestamps.get(cloud_name, 0):
            return

        # retrieve the point clouds once
        if self.db.get(['item', name, 'point_cloud']) is None:
            self.requests.extend([
                (-1, ['item', name, 'point_cloud']),
            ])
            return

        self.timestamps[cloud_name] = stamp

        logger.debug('updating {} point cloud'.format(name))

        location = self.db.get(['item', name, 'location'])
        if location == 'shelf':
            reference = ['shelf', 'pose']
        elif location in ['stow_tote', 'stow tote']:
            reference = ['tote', 'stow', 'pose']
        else:
            reference = []

        self._update_point_cloud(
            cloud_name,
            [reference, ['item', name, 'pose']],
            ['item', name, 'point_cloud'],
            ['item', name, 'point_cloud_color'],
        )

    def _update_camera(self, name):
        cloud_name = '{}_pc'.format(name)

        # check if point cloud is modified
        stamp = self.db.get(['camera', name, 'timestamp'])
        if stamp <= self.timestamps.get(cloud_name, 0):
            return

        # retrieve the point clouds once
        if self.db.get(['camera', name, 'point_cloud']) is None:
            self.requests.extend([
                (-1, ['camera', name, 'point_cloud']),
                (-1, ['camera', name, 'aligned_image'])
            ])
            return

        self.timestamps[cloud_name] = stamp

        logger.debug('updating {} point cloud'.format(name))

        self._update_point_cloud(
            cloud_name,
            [['camera', name, 'pose']],
            ['camera', name, 'point_cloud'],
            ['camera', name, 'aligned_image']
        )

    def _update_point_cloud(self, name, pose_urls, xyz_url, rgb_url=None):
        cloud = self.point_clouds.get(name)
        if not cloud:
            # construct and register a new point cloud
            cloud = PointCloudRender()
            self.program.post_drawables.append(cloud)
            self.point_clouds[name] = cloud

        options = self.options.get(cloud, {})

        # query point cloud XYZ
        cloud_xyz = self.db.get(xyz_url)
        cloud_rgb = None

        if cloud_xyz is not None:
            if len(cloud_xyz.shape) == 3:
                # mask out invalid pixels
                mask = cloud_xyz[:, :, 2] > 0
                cloud_xyz = cloud_xyz[mask]
            else:
                mask = None

            color_mode = options.get('color_mode', 'rgb')

            if color_mode == 'rgb':
                # query aligned color image to colorize point cloud
                if rgb_url:
                    cloud_rgb = self.db.get(rgb_url)
                if cloud_rgb is not None:
                    if mask is not None:
                        cloud_rgb = cloud_rgb[mask]
                else:
                    # fall back to depth colorizing
                    color_mode = 'depth'
                    logger.warn('fallback to depth color for point cloud {}'.format(name))

            if color_mode == 'depth':
                # generate colormap within range
                (llimit, ulimit) = options.get('depth_range', (0.5, 1))
                colormap = interp1d(numpy.linspace(llimit, ulimit, len(cm.viridis.colors)), cm.viridis.colors, axis=0)
                # apply color map
                cloud_rgb = colormap(cloud_xyz[:, 2].clip(llimit, ulimit))

            if color_mode == 'solid':
                color = options.get('color', (0, 255, 0))
                # color all points the same
                cloud_rgb = numpy.full(cloud_xyz.shape[:-1] + (len(color),), color)

        # build the pose
        world_pose = None
        for url in pose_urls:
            pose = self.db.get(url)
            if pose is None:
                world_pose = None
                break
            elif world_pose is None:
                world_pose = pose
            else:
                world_pose = world_pose.dot(pose)

        # perform the update
        cloud.update(xyz=cloud_xyz, rgb=cloud_rgb, pose=world_pose)

        # clear the point cloud for next update
        if xyz_url:
            self.db.put(xyz_url, None)
        if rgb_url:
            self.db.put(rgb_url, None)

if __name__ == '__main__':
    from PyQt4.QtGui import QApplication
    app = QApplication([])
    app.setApplicationName('ARC Reactor')

    window = WorldViewerWindow()
    window.show()

    from .sync import exec_async
    exec_async(app, [window], db_period=33)
