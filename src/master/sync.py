'''
Helper classes and methods for asynchronous database use in a Qt event loop.
'''

import logging

from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.gen import coroutine

from pensive.core import Store
from pensive.client_async import PensiveClientAsync

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class AsyncUpdateHandler(object):
    '''
    Helper class for supporting asynchronous database use.

    The list of synchronized database URLs is given in the list
    `self.requests`.  When the multi get of that list completes,
    `self.db` is updated and `self.update()` is called.
    '''

    ONESHOT = -1

    def __init__(self, callback, period=100, loop=None):
        self._db = Store()
        self._store = None

        self._callback = callback

        self._count = 0

        self._timer = PeriodicCallback(self._handler, period, loop)
        self._timer.start()

        self.requests = []

    def __del__(self):
        self._timer.stop()

    def request_once(self, url):
        '''
        Request `url` once.
        '''
        self.request(url, AsyncUpdateHandler.ONESHOT)

    def request(self, url, divisor=1):
        '''
        Request `url` every `divisor` update cycles.
        '''
        self.requests.append((divisor, url))

    def request_many(self, requests):
        '''
        Add multiples requests as list of `url`s or tuples of `(divisor, url)`.
        '''
        self.requests.extend(requests)

    @coroutine
    def _access_store(self):
        if not self._store:
            # access and cache the store
            self._store = yield PensiveClientAsync().default()

    @coroutine
    def _handler(self):
        gets = []
        removals = []
        for (i, request) in enumerate(self.requests):
            # check if skip period is specified
            if isinstance(request, (list, tuple)):
                (skips, path) = request

                append = False
                # check if one shot
                if skips < 1:
                    append = True
                    removals.append(i)
                # regular skipped update
                elif self._count % skips == 0:
                    append = True

                if append:
                    gets.append(path)
            else:
                gets.append(request)

        # remove all the oneshot requests in reverse order
        for i in reversed(removals):
            del self.requests[i]

        # join all list paths until multi_get accepts list-based keys
        for i in range(len(gets)):
            if not isinstance(gets[i], basestring):
                gets[i] = Store.SEPARATOR.join(gets[i])

        # remove duplicates
        gets = list(set(gets))

        try:
            if gets:
                yield self._access_store()
                result = yield self._store.multi_get(gets)
            else:
                result = {}
        except Exception:
            logger.exception('UI get failed')
        else:
            # build a local store of just the queried items so that
            # the UI code can use the nice `Store` interfaces
            for (key, value) in result.iteritems():
                if value is not None:
                    self._db.put(key, value)

            self._callback(self._db)
            self._count += 1

    @coroutine
    def _multi_put_handler(self, keys):
        try:
            yield self._access_store()
            yield self._store.multi_put(keys)
        except Exception:
            logger.exception('UI put failed: {}'.format(keys))

    def put(self, key, value=None):
        '''
        Helper to run an asynchronous `put()`.
        '''
        logger.info('set {} -> {}'.format(key, value))
        IOLoop.current().add_callback(lambda: self._multi_put_handler({key: value}))

    def multi_put(self, keys):
        '''
        Helper to run an asynchronous `multi_put()`.
        '''
        logger.info('set {}'.format(keys))
        IOLoop.current().add_callback(lambda: self._multi_put_handler(keys))

def _make_qt_handler(app, loop):
    # IOLoop handler for processing Qt events
    def _handle_events():
        app.processEvents()
        if not any([window.isVisible() for window in app.topLevelWidgets()]):
            loop.stop()

    return _handle_events

def _make_signal_handler(app):
    # handler to close all windows
    def _close_windows(signal, frame):
        logger.info('closing all windows on interrupt')
        app.closeAllWindows()

    return _close_windows

def exec_async(app, loop=None, qt_period=10):
    '''
    Run the given Qt application until all of the Qt windows close.
    '''
    loop = loop or IOLoop.current()

    # process Qt events at 100 Hz
    PeriodicCallback(_make_qt_handler(app, loop), qt_period).start()

    # install the KeyboardInterrupt handler
    import signal
    signal.signal(signal.SIGINT, _make_signal_handler(app))

    # start servicing events
    loop.start()
