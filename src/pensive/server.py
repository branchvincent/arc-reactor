'''
Web server interface for Pensive.

The server uses the following URL structure.
 * `/d/<path>`: access the default store
 * `/i/<instance>/<path>`: access the named store `instance`
 * `/s/<instance>`: control the named store `instance`

When accessing a store, the HTTP verbs GET, POST, and DELETE are mapped to
`get()`, `put()`, and `delete()`, respectively. The response to a GET is
a JSON-encoded dictionary of the form `{"data": <value>}`. Similarly,
the request body for a POST should be a JSON-encoded dictionary of the form
`{"data": <value>}`.

'''

import logging

from tornado.ioloop import IOLoop
from tornado.web import RequestHandler, Application
from tornado.escape import json_decode

from .core import Store

logger = logging.getLogger(__name__)

class StoreManager(object):
    '''
    Container class for a dictionary of names to `Store`.

    The default `Store` has key of `None`.
    '''
    def __init__(self):
        self.stores = {
            None: Store()
        }

class StoreHandler(RequestHandler):
    '''
    Handler for GET, POST, and DELETE for accessing the `Store`.
    '''

    def initialize(self):
        '''
        Initialize the handler with the applications `StoreManager`.
        '''
        self.manager = self.application.manager

    def get(self, path, instance=None):
        '''
        Handle GET requests.
        '''
        try:
            store = self.manager.stores[instance]
        except KeyError:
            logger.warn('unrecognized instance: {}'.format(instance))
            self.send_error(400, reason='unrecognized instance')
        else:
            self.write({'data': store.get(path)})

    def post(self, path, instance=None):
        '''
        Handle POST requests.
        '''
        try:
            store = self.manager.stores[instance]
        except KeyError:
            logger.warn('unrecognized instance: {}'.format(instance))
            self.send_error(400, reason='unrecognized instance')
        else:
            try:
                data = json_decode(self.request.body)['data']
            except (ValueError, KeyError) as e:
                logger.warn('malformed payload: {}\n{}'.format(e, self.request.body))
                self.send_error(400, reason='malformed payload')
            else:
                store.put(path, data)

    def delete(self, path, instance=None):
        '''
        Handle DELETE requests.
        '''
        try:
            store = self.manager.stores[instance]
        except KeyError:
            logger.warn('unrecognized instance: {}'.format(instance))
            self.send_error(400, reason='unrecognized instance')
        else:
            store.delete(path)

class PensiveServer(Application):
    '''
    Tornado web application for database access over HTTP.
    '''
    def __init__(self, port, address=''):
        super(PensiveServer, self).__init__()
        self._port = port
        self._address = address

        self.manager = StoreManager()

        # install handlers for various URLs
        self.add_handlers(r'.*', [(r'/d/(?P<path>.*)', StoreHandler)])
        self.add_handlers(r'.*', [(r'/i/(?P<instance>.+)/(?P<path>.*)', StoreHandler)])

    def log_request(self, handler):
        '''
        Log the request and response information to module logger.
        '''
        # choose the severity level based on HTTP status codes
        if handler.get_status() < 400:
            log = logger.info
        elif handler.get_status() < 500:
            log = logger.warning
        else:
            log = logger.error

        log('{} {} {} {} {} {:.2f}ms'.format(
            handler.request.remote_ip,
            handler.get_current_user() or '-',
            handler.request.method,
            handler.request.uri,
            handler.get_status(),
            1000 * handler.request.request_time())
        )

    def run(self, loop=None):
        '''
        Start servicing the Tornado event loop.
        '''

        if not loop:
            loop = IOLoop()

        loop.make_current()

        # bind the socket
        self.listen(self._port, self._address)
        logger.info('Pensive started on {}:{}'.format(
            self._address or '*', self._port))

        try:
            loop.start()
        except KeyboardInterrupt:
            pass

        loop.stop()
        loop.close()

        logger.info('Pensive stopped')
