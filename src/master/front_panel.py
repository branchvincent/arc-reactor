'''
Front panel user interface for master controller.
'''

import logging

from time import time

from PySide.QtGui import QMainWindow, QCheckBox
from PySide.QtCore import QTimer

from master.pickfsm import PickStateMachine

from .ui.front_panel import Ui_FrontPanel
from .sync import AsyncUpdateMixin

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

RUN_MODES = ['step_once', 'run_once', 'run_all', 'full_auto']
JOB_TYPES = ['pick', 'stow', 'final']

def _call(target, *args, **kwargs):
    def _cb():
        return target(*args, **kwargs)
    return _cb

class FrontPanel(QMainWindow, AsyncUpdateMixin):
    '''
    Main window for the front panel.
    '''

    def __init__(self):
        super(FrontPanel, self).__init__()

        self.ui = Ui_FrontPanel()
        self.ui.setupUi(self)

        self.pick = None

        self.setup_async()

        # install run mode click handlers
        for mode in ['step_once', 'run_once', 'run_all']:
            self._ui('run_' + mode).clicked.connect(_call(self.put, '/robot/run_mode', mode))
        self.ui.run_full_auto.clicked.connect(_call(self.multi_put, {
            '/robot/run_mode': 'full_auto',
            '/checkpoint': None,
            '/fault': None,
            '/simulate': None
        }))

        for mode in JOB_TYPES:
            self._ui('job_' + mode).clicked.connect(_call(self.put, '/robot/task', mode))

        self.ui.mc_run.clicked.connect(_call(self._run_handler))
        self.ui.mc_reset.clicked.connect(_call(self._reset_handler))

        self.requests = [
            '/robot/run_mode',
            '/robot/task',
            '/checkpoint',
            '/fault',
            '/simulate',
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
            'plan_route',
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
            'vacuum',
            'cameras',
            'object_detection',
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

    def _run_handler(self):
        if not self.pick:
            self._reset_handler()

        run_mode = self.db.get('/robot/run_mode')
        logger.info('running in mode {}'.format(run_mode))

        if run_mode == 'step_once':
            self.pick.runStep()
        elif run_mode == 'run_once':
            self.pick.runOrdered(self.pick.getCurrentState())
        elif run_mode in ['run_all', 'full_auto']:
            while not self.pick.doneOrderFile():
                self.pick.runOrdered(self.pick.getCurrentState())
        else:
            logger.error('unimplemented run mode: "{}"'.format(run_mode))

    def _reset_handler(self):
        logger.info('reset state machine')

        self.pick = PickStateMachine()
        self.pick.loadStates()
        self.pick.setupTransitions()
        self.pick.setCurrentState('si') #always start with SelectItem

        job_type = self.db.get('/robot/task')
        if job_type == 'pick':
            logger.info('load order file for pick task')
            self.pick.loadOrderFile('test/master/order_test.json')
        else:
            logger.error('unimplemented job type: "{}"'.format(job_type))


    def update(self):
        '''
        Main update handler for the UI.
        '''

        now = time()

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
        run_mode = self.db.get('/robot/run_mode')
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
        job_type = self.db.get('/robot/task')
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
            if self.db.get(url + '/timestamp', default=0) < now - 2:
                self._ui(name).setStyleSheet(alarm_button_style)
            else:
                self._ui(name).setStyleSheet('color: #008000; font-weight: bold;')

        # update toggle UIs
        self._update_toggle_list(self.db, self.checkpoints, '/checkpoint/')
        self._update_toggle_list(self.db, self.faults, '/fault/')
        self._update_toggle_list(self.db, self.simulations, '/simulate/')

        alert = any([self.db.get('/checkpoint/{}'.format(k), 0) for k in self.checkpoints])
        self.ui.checkpoints_label.setStyleSheet(alert_style if alert else '')

        alert = any([self.db.get('/fault/{}'.format(k), 0) for k in self.faults])
        self.ui.faults_label.setStyleSheet(alert_style if alert else '')

        alert = any([self.db.get('/simulate/{}'.format(k), 0) for k in self.simulations])
        self.ui.simulation_label.setStyleSheet(alert_style if alert else '')

    def _put_checkbox(self, key, checkbox):
        self.put(key, checkbox.isChecked())
        checkbox.blockSignals(True)
        checkbox.setChecked(not checkbox.isChecked())
        checkbox.blockSignals(False)

    def _ui(self, name):
        return getattr(self.ui, name)

if __name__ == '__main__':
    from PySide.QtGui import QApplication, QStyleFactory

    app = QApplication([])
    app.setApplicationName('ARC Reactor')
    app.setStyle(QStyleFactory.create('fusion'))

    window = FrontPanel()
    window.show()

    from .sync import exec_async
    exec_async(app, [window])
