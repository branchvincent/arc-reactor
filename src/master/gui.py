'''
Front panel user interface for master controller.
'''

import logging

from time import time

from PySide.QtGui import QMainWindow

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

        # install run mode click handlers
        for mode in RUN_MODES:
            self._ui('run_' + mode).clicked.connect(_call(self.put, '/master/run_mode', mode))

    def update(self, view):
        '''
        Main update handler for the UI.
        '''

        # style sheet for blinking foreground at 1 Hz
        if time() % 1 >= 0.5:
            alarm_style = 'color: #ff0000;'
        else:
            alarm_style = 'color: #800000;'

        # update run mode UI
        run_mode = view.get('/master/run_mode')
        if run_mode in RUN_MODES:
            self.ui.run_label.setStyleSheet('')
            for mode in RUN_MODES:
                self._ui('run_' + mode).setStyleSheet('color: red;' if run_mode == mode else '')
                self._ui('run_' + mode).setChecked(run_mode == mode)
        else:
            for mode in RUN_MODES:
                self._ui('run_' + mode).setStyleSheet('')
                self._ui('run_' + mode).setChecked(False)
            self.ui.run_label.setStyleSheet(alarm_style)

    def put(self, key, value=None):
        '''
        Helper method for dispatching a Tornado coroutine to update the database.
        '''
        IOLoop.current().add_callback(lambda: self._put(key, value))

    @coroutine
    def _put(self, key, value):
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
            result = yield store.multi_get([
                '/master/run_mode'
            ])
        except:
            logger.exception('UI update query failed')
        else:
            # build a local store of just the queried items so that
            # the UI code can use the nice `Store` interfaces
            for (key, value) in result.iteritems():
                view.put(key, value)

        # update the UI
        window.update(view)

    # process Qt events at 100 Hz
    PeriodicCallback(handle_events, 10).start()
    # update the UI at 10 Hz
    PeriodicCallback(update_ui, 100).start()

    # start servicing events
    IOLoop.current().start()
