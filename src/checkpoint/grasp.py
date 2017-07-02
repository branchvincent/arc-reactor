import logging

from math import pi
import numpy

from klampt.vis import GLRealtimeProgram, gldraw

from PyQt4.QtGui import QMainWindow, QLabel, QPushButton

from master.world import build_world, rpy, numpy2klampt
from master.vis import PointCloud

from states.common.capture_photo import NoViewingCameraError

from util.math_helpers import normalize, rotate

from .ui.grasp import Ui_GraspWindow

logger = logging.getLogger(__name__)

def _call(target, *args, **kwargs):
    def _cb(*args2, **kwargs2):
        return target(*args, **kwargs)
    return _cb

class WorldViewer(GLRealtimeProgram):
    def __init__(self, store, photo_urls, grasps):
        GLRealtimeProgram.__init__(self, 'Grasp Checkpoint')

        self.world = build_world(store)

        self.fps = 30.0
        self.dt = 1 / self.fps

        self.pcs = []
        for photo_url in photo_urls:
            point_cloud = store.get(photo_url + ['point_cloud_segmented'])
            camera_pose = store.get(photo_url + ['pose'])
            labeled_image = store.get(photo_url + ['labeled_image'])
            full_color = store.get(photo_url + ['full_color'])

            mask = labeled_image > 0

            pc = PointCloud()
            pc.update(point_cloud[mask], full_color[mask], camera_pose)
            self.pcs.append(pc)

        self.grasps = grasps

        target_grasp = store.get('/robot/target_grasp', {})
        for (i, grasp) in enumerate(self.grasps):
            try:
                if all([grasp[x] == target_grasp[x] for x in ['location', 'camera', 'index']]):
                    self.selected_grasp_index = i
                    break
            except KeyError:
                continue
        else:
            self.selected_grasp_index = None

        self.show_grasp_index = self.selected_grasp_index

        self.view.camera.rot = [0, pi/4, pi/2 + pi/4]

    def display(self):
        self.world.drawGL()

        for pc in self.pcs:
            pc.draw()

        for (i, grasp) in enumerate(self.grasps):
            pose = numpy.eye(4)
            # normal vector points along Z
            pose[:3, 2] = normalize(grasp['orientation'])
            pose[:3, 0] = normalize(numpy.cross(rotate(rpy(pi / 2, 0, 0), pose[:3, 2]), pose[:3, 2]))
            pose[:3, 1] = normalize(numpy.cross(pose[:3, 2], pose[:3, 0]))
            # position is grasp center
            pose[:3, 3] = grasp['center']

            show = self.show_grasp_index
            if show is None:
                show = self.selected_grasp_index

            scale = 1 if i == show else 0.5
            gldraw.xform_widget(numpy2klampt(pose), 0.1 * scale, 0.01 * scale, fancy=True)

    def idle(self):
        pass

    def mousefunc(self,button,state,x,y):
        return GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLRealtimeProgram.motionfunc(self,x,y,dx,dy)

class WorldViewerWindow(QMainWindow):
    def __init__(self, store, locations):
        super(WorldViewerWindow, self).__init__()

        self.store = store
        self.grasps = []
        self.photo_urls = []

        for location in locations:
            available_cameras = self.store.get(['system', 'viewpoints', location], [])

            # use end-effector camera for bins and fixed cameras otherwise
            if 'tcp' in available_cameras:
                if 'bin' in location:
                    available_cameras = ['tcp']
                else:
                    available_cameras.remove('tcp')

            if not available_cameras:
                raise NoViewingCameraError('no camera available for location {}'.format(location))

            for camera in available_cameras:
                photo_url = ['photos', location, camera]
                self.photo_urls.append(photo_url)
                self.grasps.extend(self.store.get(photo_url + ['vacuum_grasps']))

        self.grasps.sort(key=lambda g: -g['score'])

        self.ui = Ui_GraspWindow()
        self.ui.setupUi(self)

        self.program = WorldViewer(self.store, self.photo_urls, self.grasps)
        self.ui.view.setProgram(self.program)
        self.setWindowTitle(self.program.name)
        self.ui.view.setMaximumSize(1920, 1080)
        self.ui.view.enterEvent = _call(self.set_show, None)

        self.select_buttons = []
        self.name_labels = []

        for (i, grasp) in enumerate(self.grasps):
            c = 0
            widgets = []

            label = QLabel('{0}-{1}-{2}'.format(
                grasp.get('location', '?'),
                grasp.get('camera', '?'),
                grasp.get('index', '?'),
            ))
            widgets.append(label)
            self.name_labels.append(label)
            c += 1

            label = QLabel('{:.2f}'.format(grasp['score']))
            label.setStyleSheet('font-weight: bold;')
            widgets.append(label)
            c += 1

            label = QLabel('({:.3f}, {:.3f} {:.3f})'.format(*list(grasp['center'].flat)))
            widgets.append(label)
            c += 1

            select_button = QPushButton('Select')
            select_button.setCheckable(True)
            select_button.clicked.connect(_call(self.select_grasp, i))
            self.select_buttons.append(select_button)
            widgets.append(select_button)
            c += 1

            good_button = QPushButton('Good')
            bad_button = QPushButton('Bad')

            good_button.setCheckable(True)
            good_button.clicked.connect(_call(self.set_evaluation, good_button, bad_button, i, True))
            widgets.append(good_button)
            c += 1

            bad_button.setCheckable(True)
            bad_button.clicked.connect(_call(self.set_evaluation, good_button, bad_button, i, False))
            widgets.append(bad_button)
            c += 1

            good_button.setStyleSheet('color: green;')
            bad_button.setStyleSheet('color: red;')
            if grasp.get('good') == True:
                good_button.setChecked(True)
                good_button.setStyleSheet('color: green; font-weight: bold')
                bad_button.setStyleSheet('color: red;')
            elif grasp.get('good') == False:
                bad_button.setChecked(True)
                good_button.setStyleSheet('color: green;')
                bad_button.setStyleSheet('color: red; font-weight: bold')

            for (c, widget) in enumerate(widgets):
                widget.enterEvent = _call(self.set_show, i)
                self.ui.grasp_panel_layout.addWidget(widget, i, c, 1, 1)

        self.select_grasp(self.program.selected_grasp_index)

    def set_show(self, index, force=False):
        self.program.show_grasp_index = index

        for (i, label) in enumerate(self.name_labels):
            if i == index:
                label.setStyleSheet('background-color: yellow;')
            else:
                label.setStyleSheet('')

    def select_grasp(self, index):
        self.program.selected_grasp_index = index

        for (i, button) in enumerate(self.select_buttons):
            button.setChecked(i == index)

            if button.isChecked():
                button.setStyleSheet('color: blue; font-weight: bold')
            else:
                button.setStyleSheet('')

        if index is not None:
            target_grasp = self.grasps[index]
            self.store.put('/robot/target_grasp', target_grasp)
            logger.info('set target grasp to {}-{}-{}'.format(
                target_grasp.get('location', '?'),
                target_grasp.get('camera', '?'),
                target_grasp.get('index', '?')
            ))

        self.set_show(index, force=True)

    def set_evaluation(self, good_button, bad_button, index, good):
        good_button.setChecked(good)
        bad_button.setChecked(not good)

        if good:
            good_button.setStyleSheet('color: green; font-weight: bold')
            bad_button.setStyleSheet('color: red;')
        else:
            good_button.setStyleSheet('color: green;')
            bad_button.setStyleSheet('color: red; font-weight: bold')

        eval_grasp = self.grasps[index]

        if eval_grasp.get('good') != good:
            eval_grasp['good'] = good

            if all([x in eval_grasp for x in ['location', 'camera', 'index']]):
                grasps = self.store.get(['photos', eval_grasp['location'], eval_grasp['camera'], 'vacuum_grasps'])
                grasps[eval_grasp['index']]['good'] = good
                self.store.put(['photos', eval_grasp['location'], eval_grasp['camera'], 'vacuum_grasps'], grasps)
                logger.info('marked grasp {}-{}-{} as {}'.format(
                    eval_grasp.get('location', '?'),
                    eval_grasp.get('camera', '?'),
                    eval_grasp.get('index', '?'),
                    'good' if good else 'bad'
                ))

def run(locations=None, modal=True, store=None):
    from pensive.client import PensiveClient
    store = store or PensiveClient().default()

    locations = locations or store.get('/robot/target_locations', [])

    from PyQt4.QtGui import QApplication
    app = QApplication.instance()
    if app:
        embedded = True
    else:
        embedded = False
        app = QApplication([])
        app.setApplicationName('ARC Reactor')

    window = WorldViewerWindow(store, locations)
    if modal:
        from PyQt4.QtCore import Qt
        window.setWindowModality(Qt.ApplicationModal)
    window.show()

    if not embedded:
        app.exec_()

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='grasp checkpoint', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-p', '--path', metavar='PATH', help='path to a JSON database')
    parser.add_argument('locations', nargs='*', metavar='LOCATION', help='list of locations to show')

    args = parser.parse_args()

    if args.path:
        # read the file
        if args.path.endswith('.gz'):
            import gzip
            data = gzip.open(args.path, 'rb').read()
        else:
            data = open(args.path).read()

        # load the JSON object
        from pensive.client import json_decode
        obj = json_decode(data)

        # populate in-memory store
        from pensive.core import Store
        store = Store(obj)

    else:
        # connect to the database
        from pensive.client import PensiveClient
        client = PensiveClient(args.address)

        # get the store
        store = client.store(args.store)

    run(args.locations, store=store)
