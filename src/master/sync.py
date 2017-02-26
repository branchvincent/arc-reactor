'''
Helper classes and methods for asynchronous database use in a Qt event loop.
'''

import logging

from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.gen import coroutine

from pensive.core import Store
from pensive.client_async import PensiveClientAsync

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class AsyncUpdateMixin(object):
    '''
    Helper class for supporting asynchronous database use.

    The list of synchronized database URLs is given in the list
    `self.requests`.  When the multi get of that list completes,
    `self.db` is updated and `self.update()` is called.

    Call `self.setup_async()` to initialize.
    '''

    def setup_async(self):
        '''
        Initialize `self.db`.
        '''
        self.db = Store()
        self._store = None

    @coroutine
    def _access_store(self):
        if not self._store:
            # access and cache the store
            self._store = yield PensiveClientAsync().default()

    @coroutine
    def _update_handler(self):
        try:
            yield self._access_store()
            result = yield self._store.multi_get(self.requests)
        except:
            logger.exception('UI get failed')
        else:
            # build a local store of just the queried items so that
            # the UI code can use the nice `Store` interfaces
            for (key, value) in result.iteritems():
                if value is not None:
                    self.db.put(key, value)

            self.update()

    @coroutine
    def _multi_put_handler(self, keys):
        try:
            yield self._access_store()
            yield self._store.multi_put(keys)
        except:
            logger.exception('UI put failed: {}'.format(keys))

    def update(self):
        '''
        Handler when an asynchronous update completes.
        '''
        pass

    def put(self, key, value=None):
        '''
        Helper to run an asynchronous `put()`.
        '''
        logger.info('set {} -> {}'.format(key, value))
        IOLoop.current().add_callback(lambda: self._multi_put_handler({key: value}))

    def multi_put(self, keys):
        '''
        Helper to run an asynchronous `mulit_put()`.
        '''
        logger.info('set {}'.format(keys))
        IOLoop.current().add_callback(lambda: self._multi_put_handler(keys))

def _make_qt_handler(app, windows, loop):
    # IOLoop handler for processing Qt events
    def _handle_events():
        app.processEvents()
        if not any([window.isVisible() for window in windows]):
            loop.stop()

    return _handle_events

def exec_async(app, windows=None, loop=None, qt_period=10, db_period=100):
    '''
    Run the given Qt application until all of the Qt windows close.
    '''
    windows = windows or []
    loop = loop or IOLoop.current()

    # process Qt events at 100 Hz
    PeriodicCallback(_make_qt_handler(app, windows, loop), qt_period).start()

    for window in windows:
        if isinstance(window, AsyncUpdateMixin):
            # update the UI at 10 Hz
            PeriodicCallback(window._update_handler, db_period).start()

    # start servicing events
    loop.start()
