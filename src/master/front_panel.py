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

from master.pickfsm import PickStateMachine

logger = logging.getLogger(__name__)

RUN_MODES = ['step_once', 'run_once', 'run_all', 'full_auto']
JOB_TYPES = ['pick', 'stow', 'final']

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
    
        self.pick = PickStateMachine()
        self.pick.loadStates()
        self.pick.setupTransitions()

        self.view = Store()

        # install run mode click handlers
        for mode in ['step_once', 'run_once', 'run_all']:
            self._ui('run_' + mode).clicked.connect(_call(self._put, '/robot/run_mode', mode))
        self.ui.run_full_auto.clicked.connect(_call(self._multi_put, {
            '/robot/run_mode': 'full_auto',
            '/checkpoint': None,
            '/fault': None,
            '/simulate': None
        }))

        for mode in JOB_TYPES:
            self._ui('job_' + mode).clicked.connect(_call(self._put, '/robot/task', mode))
            if mode == 'pick':
                self.pick.loadOrderFile('test/master/order_test.json') #HERE

        self.requests = [
            '/robot/run_mode',
            '/checkpoint',
            '/fault',
            '/simulate',
            '/robot/task',
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

        self.checkpoints = self._make_path_map([
            'select_item',
            'motion_plan',
            'plan_execution',

        ])
        self._load_toggle_list('/checkpoint/', self.checkpoints, self.ui.checkpointsLayout)

        self.faults = self._make_path_map([
            'cam_top_right_lost',
            'cam_top_left_lost',
            'cam_bottom_right_lost',
            'cam_bottom_left_lost',
            'cam_box_lost',
            'cam_tote_lost',
            'cam_box_lost',
            'robot_fail',
            'gripper_fail',
            'plan_route_fail',
            'scale_tote_wrong',
            'scale_shelf_wrong',
        ])
        self._load_toggle_list('/fault/', self.faults, self.ui.faultsLayout)

        self.simulations = self._make_path_map([
            'robot_motion',
            'vacuum'
        ])
        self._load_toggle_list('/simulate/', self.simulations, self.ui.simulationLayout)

        self.update()

        # refresh timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(lambda: self.update())
        self.timer.start(1000 / 30.0)

    def _make_path_map(self, paths):
        return dict(zip(paths, [None] * len(paths)))

    def _load_toggle_list(self, prefix, paths, layout):
        for name in reversed(sorted(paths)):
            checkbox = QCheckBox()
            checkbox.setText(name.replace('_', ' ').title())
            checkbox.setObjectName(name)

            paths[name] = checkbox
            layout.insertWidget(0, checkbox)

            checkbox.stateChanged.connect(_call(self._put_checkbox, prefix + name, checkbox))

    def _update_toggle_list(self, view, paths, prefix):
        for (name, checkbox) in paths.iteritems():
            checkbox.blockSignals(True)
            checkbox.setChecked(view.get(prefix + name, default=False))
            checkbox.blockSignals(False)
            checkbox.setEnabled(view.get('/robot/run_mode') != 'full_auto')

    def update(self, view=None):
        '''
        Main update handler for the UI.
        '''

        now = time()

        if not view:
            view = Store()

        # style sheet for blinking foreground at 1 Hz
        if now % 1 >= 0.5:
            alarm_label_style = 'color: #ffffff; background-color: #ff0000;'
            alarm_button_style = 'color: #ff0000;'
            alert_style = 'background-color: #ffff00;'
        else:
            alarm_label_style = 'color: #ffffff; background-color: #800000;'
            alarm_button_style = 'color: #800000;'
            alert_style = ''

        # update run mode UI
        run_mode = view.get('/robot/run_mode')
        if run_mode in RUN_MODES:
            self.ui.run_label.setStyleSheet('')
            for mode in RUN_MODES:
                self._ui('run_' + mode).setStyleSheet('color: #000080;' if run_mode == mode else '')
                self._ui('run_' + mode).setChecked(run_mode == mode)
        else:
            for mode in RUN_MODES:
                self._ui('run_' + mode).setStyleSheet('')
                self._ui('run_' + mode).setChecked(False)
            self.ui.run_label.setStyleSheet(alarm_label_style)

        # update job type UI
        job_type = view.get('/robot/task')
        if job_type in JOB_TYPES:
            self.ui.job_label.setStyleSheet('')
            for mode in JOB_TYPES:
                self._ui('job_' + mode).setStyleSheet('color: #000080;' if job_type == mode else '')
                self._ui('job_' + mode).setChecked(job_type == mode)
        else:
            for mode in JOB_TYPES:
                self._ui('job_' + mode).setStyleSheet('')
                self._ui('job_' + mode).setChecked(False)
            self.ui.job_label.setStyleSheet(alarm_label_style)

        # update hardware status UI
        for (name, url) in self.hardware_map.iteritems():
            if view.get(url + '/timestamp', default=0) < now - 2:
                self._ui(name).setStyleSheet(alarm_button_style)
            else:
                self._ui(name).setStyleSheet('color: #008000; font-weight: bold;')

        # update toggle UIs
        self._update_toggle_list(view, self.checkpoints, '/checkpoint/')
        self._update_toggle_list(view, self.faults, '/fault/')
        self._update_toggle_list(view, self.simulations, '/simulate/')

        alert = any([view.get('/checkpoint/{}'.format(k), 0) for k in self.checkpoints])
        self.ui.checkpoints_label.setStyleSheet(alert_style if alert else '')

        alert = any([view.get('/fault/{}'.format(k), 0) for k in self.faults])
        self.ui.faults_label.setStyleSheet(alert_style if alert else '')

        alert = any([view.get('/simulate/{}'.format(k), 0) for k in self.simulations])
        self.ui.simulation_label.setStyleSheet(alert_style if alert else '')

    def _put_checkbox(self, key, checkbox):
        self._put(key, checkbox.isChecked())
        checkbox.blockSignals(True)
        checkbox.setChecked(not checkbox.isChecked())
        checkbox.blockSignals(False)

    def _put(self, key, value=None):
        logger.info('set {} -> {}'.format(key, value))
        IOLoop.current().add_callback(lambda: self._put_handler(key, value))

    def _multi_put(self, keys):
        logger.info('set {}'.format(keys))
        IOLoop.current().add_callback(lambda: self._multi_put_handler(keys))

    @coroutine
    def _put_handler(self, key, value):
        try:
            store = yield PensiveClientAsync().default()
            yield store.put(key, value)
        except:
            logger.exception('UI put failed: {} -> {}'.format(key, value))
            # TODO: notify the user that the operation failed

    @coroutine
    def _multi_put_handler(self, keys):
        try:
            store = yield PensiveClientAsync().default()
            yield store.multi_put(keys)
        except:
            logger.exception('UI multi put failed: {}'.format(keys))
            # TODO: notify the user that the operation failed

    def _ui(self, name):
        return getattr(self.ui, name)

if __name__ == '__main__':
    from PySide.QtGui import QApplication, QStyleFactory

    app = QApplication([])
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
