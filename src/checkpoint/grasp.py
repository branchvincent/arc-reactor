import logging

from math import pi
import numpy

from klampt.vis import GLRealtimeProgram, gldraw

from PyQt4.QtGui import QMainWindow, QLabel, QPushButton

from master.world import build_world, rpy, numpy2klampt
from master.vis import PointCloud

from util.math_helpers import normalize, rotate

from .ui.grasp import Ui_GraspWindow

logger = logging.getLogger(__name__)

def _call(target, *args, **kwargs):
    def _cb():
        return target(*args, **kwargs)
    return _cb

class WorldViewer(GLRealtimeProgram):
    def __init__(self, store, photo_url):
        GLRealtimeProgram.__init__(self, 'Grasp Checkpoint')

        self.world = build_world(store)

        self.fps = 30.0
        self.dt = 1 / self.fps

        self.select_grasp_index = store.get('/robot/target_grasp', {'index': None})['index']
        self.show_grasp_index = self.select_grasp_index

        point_cloud = store.get(photo_url + ['point_cloud_segmented'])
        camera_pose = store.get(photo_url + ['pose'])
        labeled_image = store.get(photo_url + ['labeled_image'])
        full_color = store.get(photo_url + ['full_color'])

        mask = labeled_image > 0

        self.pc = PointCloud()
        self.pc.update(point_cloud[mask], full_color[mask], camera_pose)

        self.grasps = store.get(photo_url + ['vacuum_grasps'])

    def display(self):
        self.world.drawGL()

        self.pc.draw()

        for (i, grasp) in enumerate(self.grasps):
            pose = numpy.eye(4)
            # normal vector points along Z
            pose[:3, 2] = normalize(grasp['orientation'])
            pose[:3, 0] = normalize(numpy.cross(rotate(rpy(pi / 2, 0, 0), pose[:3, 2]), pose[:3, 2]))
            pose[:3, 1] = normalize(numpy.cross(pose[:3, 2], pose[:3, 0]))
            # position is grasp center
            pose[:3, 3] = grasp['center']

            scale = 1 if i == self.show_grasp_index else 0.5
            gldraw.xform_widget(numpy2klampt(pose), 0.1 * scale, 0.01 * scale, fancy=True)

    def idle(self):
        pass

    def mousefunc(self,button,state,x,y):
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

class WorldViewerWindow(QMainWindow):
    def __init__(self, store):
        super(WorldViewerWindow, self).__init__()

        self.store = store

        location = self.store.get('/robot/target_location')
        available_cameras = self.store.get(['system', 'viewpoints', location], [])

        # use end-effector camera for bins and fixed cameras otherwise
        if 'tcp' in available_cameras:
            if 'bin' in location:
                available_cameras = ['tcp']
            else:
                available_cameras.remove('tcp')

        if not available_cameras:
            raise RuntimeError('no camera available for {}'.format(location))

        # TODO: choose camera a better way
        camera = available_cameras[0]
        self.photo_url = ['photos', location, camera]

        self.grasps = self.store.get(self.photo_url + ['vacuum_grasps'])

        self.ui = Ui_GraspWindow()
        self.ui.setupUi(self)

        self.program = WorldViewer(self.store, self.photo_url)
        self.ui.view.setProgram(self.program)
        self.setWindowTitle(self.program.name)
        self.ui.view.setMaximumSize(1920, 1080)

        self.select_buttons = []

        for (i, grasp) in enumerate(store.get(self.photo_url + ['vacuum_grasps'])):
            label = QLabel('{0}: ({1[0]:.3f}, {1[1]:.3f} {1[2]:.3f})'.format(
                i,
                list(grasp['center'].flat)
            ))
            self.ui.grasp_panel_layout.addWidget(label, i, 0, 1, 1)

            show_button = QPushButton('Show')
            show_button.clicked.connect(_call(self.set_show, i))
            self.ui.grasp_panel_layout.addWidget(show_button, i, 1, 1, 1)

            select_button = QPushButton('Select')
            select_button.setCheckable(True)
            select_button.clicked.connect(_call(self.select_grasp, i))
            self.select_buttons.append(select_button)
            self.ui.grasp_panel_layout.addWidget(select_button, i, 2, 1, 1)

            good_button = QPushButton('Good')
            bad_button = QPushButton('Bad')

            good_button.setStyleSheet('color: green; font-weight: bold;')
            good_button.setCheckable(True)
            good_button.clicked.connect(_call(self.set_evaluation, good_button, bad_button, i, True))
            self.ui.grasp_panel_layout.addWidget(good_button, i, 3, 1, 1)

            bad_button.setStyleSheet('color: red; font-weight: bold;')
            bad_button.setCheckable(True)
            bad_button.clicked.connect(_call(self.set_evaluation, good_button, bad_button, i, False))
            self.ui.grasp_panel_layout.addWidget(bad_button, i, 4, 1, 1)

            if grasp.get('good') == True:
                good_button.setChecked(True)
            elif grasp.get('good') == False:
                bad_button.setChecked(True)

        self.select_grasp(self.program.select_grasp_index)

    def set_show(self, index, force=False):
        if not force and self.program.show_grasp_index == index:
            self.program.show_grasp_index = None
        else:
            self.program.show_grasp_index = index

    def select_grasp(self, index):
        for (i, button) in enumerate(self.select_buttons):
            button.setChecked(i == index)

            if button.isChecked():
                button.setStyleSheet('color: blue; font-weight: bold')
            else:
                button.setStyleSheet('')

        if self.program.select_grasp_index != index:
            self.store.put('/robot/target_grasp', self.grasps[index])
            logger.info('set target grasp to {}'.format(index))

        self.set_show(index, force=True)

    def set_evaluation(self, good_button, bad_button, index, good):
        good_button.setChecked(good)
        bad_button.setChecked(not good)

        if self.grasps[index].get('good') != good:
            self.grasps[index]['good'] = good
            self.store.put(self.photo_url + ['vacuum_grasps'], self.grasps)
            logger.info('marked grasp {} as {}'.format(index, 'good' if good else 'bad'))

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
