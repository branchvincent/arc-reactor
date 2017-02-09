'''
Front panel user interface for master controller.
'''

import logging

from time import time

from PySide.QtGui import QMainWindow, QCheckBox
from PySide.QtCore import QTimer

from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.gen import coroutine

from pensive.core import Store
from pensive.client_async import PensiveClientAsync

from .ui.front_panel import Ui_FrontPanel

logger = logging.getLogger(__name__)

RUN_MODES = ['step_once', 'run_once', 'run_all', 'full_auto']

def _call(target, *args, **kwargs):
    def _cb():
        return target(*args, **kwargs)
    return _cb

class FrontPanel(QMainWindow):
    '''
    Main window for the front panel.
    '''

    def __init__(self):
        super(FrontPanel, self).__init__()

        self.ui = Ui_FrontPanel()
        self.ui.setupUi(self)

        self.view = Store()

        # install run mode click handlers
        for mode in RUN_MODES:
            self._ui('run_' + mode).clicked.connect(_call(self._put, '/master/run_mode', mode))

        self.requests = [
            '/master/run_mode',
            '/checkpoint',
            '/fault'
        ]

        self.hardware_map = {
            'hw_cam_bl': '/camera/shelf_bl',
            'hw_cam_br': '/camera/shelf_br',
            'hw_cam_tl': '/camera/shelf_tl',
            'hw_cam_tr': '/camera/shelf_tr',
            'hw_cam_box': '/camera/box',
            'hw_cam_tote': '/camera/tote',
            'hw_gripper': '/gripper',
            'hw_robot': '/robot',
            'hw_scale_shelf': '/scale/shelf',
            'hw_scale_tote': '/scale/tote',
            'hw_shelf': '/shelf',
            'hw_vacuum': '/vacuum'
        }
        for (name, url) in self.hardware_map.iteritems():
            self.requests.append(url + '/timestamp')

        self.checkpoints = {
            'select_item': None,
            'motion_plan': None,
            'plan_execution': None
        }

        for name in reversed(sorted(self.checkpoints.keys())):
            checkbox = QCheckBox()
            checkbox.setText(name.replace('_', ' ').title())
            checkbox.setObjectName(name)

            self.checkpoints[name] = checkbox
            self.ui.checkpointsLayout.insertWidget(0, checkbox)

            checkbox.stateChanged.connect(_call(self._put_checkbox, '/checkpoint/' + name, checkbox))

        self.faults = {
            'cam_top_right_lost': None,
            'cam_top_left_lost': None,
            'cam_bottom_right_lost': None,
            'cam_bottom_left_lost': None,
            'cam_box_lost': None,
            'cam_tote_lost': None,
            'cam_box_lost': None,
            'robot_fail': None,
            'gripper_fail': None,
            'plan_route_fail': None,
            'scale_tote_wrong': None,
            'scale_shelf_wrong': None,
        }

        for name in reversed(sorted(self.faults.keys())):
            checkbox = QCheckBox()
            checkbox.setText(name.replace('_', ' ').title())
            checkbox.setObjectName(name)

            self.faults[name] = checkbox
            self.ui.faultsLayout.insertWidget(0, checkbox)

            checkbox.stateChanged.connect(_call(self._put_checkbox, '/fault/' + name, checkbox))

        self.update()

        # refresh timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(lambda: self.update())
        self.timer.start(1000 / 30.0)

    def update(self, view=None):
        '''
        Main update handler for the UI.
        '''

        now = time()

        if not view:
            view = Store()

        # style sheet for blinking foreground at 1 Hz
        if now % 1 >= 0.5:
            alarm_style = 'color: #ff0000;'
        else:
            alarm_style = 'color: #800000;'

        # update run mode UI
        run_mode = view.get('/master/run_mode')
        if run_mode in RUN_MODES:
            self.ui.run_label.setStyleSheet('')
            for mode in RUN_MODES:
                self._ui('run_' + mode).setStyleSheet('color: #000080;' if run_mode == mode else '')
                self._ui('run_' + mode).setChecked(run_mode == mode)
        else:
            for mode in RUN_MODES:
                self._ui('run_' + mode).setStyleSheet('')
                self._ui('run_' + mode).setChecked(False)
            self.ui.run_label.setStyleSheet(alarm_style)

        # update hardware status UI
        for (name, url) in self.hardware_map.iteritems():
            if view.get(url + '/timestamp', default=0) < now - 2:
                self._ui(name).setStyleSheet(alarm_style)
            else:
                self._ui(name).setStyleSheet('color: #008000; font-weight: bold;')

        # update checkpoint UI
        for (name, checkbox) in self.checkpoints.iteritems():
            checkbox.blockSignals(True)
            checkbox.setChecked(view.get('/checkpoint/' + name, default=False))
            checkbox.blockSignals(False)

        # update faults UI
        for (name, checkbox) in self.faults.iteritems():
            checkbox.blockSignals(True)
            checkbox.setChecked(view.get('/fault/' + name, default=False))
            checkbox.blockSignals(False)

    def _put_checkbox(self, key, checkbox):
        self._put(key, checkbox.isChecked())
        checkbox.blockSignals(True)
        checkbox.setChecked(not checkbox.isChecked())
        checkbox.blockSignals(False)

    def _put(self, key, value=None):
        IOLoop.current().add_callback(lambda: self._put_handler(key, value))

    @coroutine
    def _put_handler(self, key, value):
        try:
            store = yield PensiveClientAsync().default()
            yield store.put(key, value)
        except:
            logger.exception('UI put failed: {} -> {}', key, value)
            # TODO: notify the user that the operation failed

    def _ui(self, name):
        return getattr(self.ui, name)

if __name__ == '__main__':
    import sys

    from PySide.QtGui import QApplication, QStyleFactory

    app = QApplication(sys.argv)
    app.setApplicationName('ARC Reactor')
    app.setStyle(QStyleFactory.create('fusion'))

    window = FrontPanel()
    window.show()

    # IOLoop handler for processing Qt events
    def handle_events():
        app.processEvents(10)
        if not window.isVisible():
            IOLoop.current().stop()

    # IOLoop handler for asynchronously querying the database
    @coroutine
    def update_ui():
        view = Store()

        try:
            # TODO: access the store once and cache it
            store = yield PensiveClientAsync().default()
            # query the database
            result = yield store.multi_get(window.requests)
        except:
            logger.exception('UI update query failed')
        else:
            # build a local store of just the queried items so that
            # the UI code can use the nice `Store` interfaces
            for (key, value) in result.iteritems():
                if value is not None:
                    view.put(key, value)

        # update the UI
        window.update(view)

    # process Qt events at 100 Hz
    PeriodicCallback(handle_events, 10).start()
    # update the UI at 10 Hz
    PeriodicCallback(update_ui, 100).start()

    # start servicing events
    IOLoop.current().start()
