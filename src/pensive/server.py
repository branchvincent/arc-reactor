'''
Web server interface for Pensive.

The server uses the following URL structure.



'''

import logging
logger = logging.getLogger(__name__)

import json

from tornado.ioloop import IOLoop
from tornado.web import RequestHandler, Application

from .core import Store

class StoreManager(object):

    def __init__(self):
        self._store = Store

class PensiveHandler(RequestHandler):

    def get(self, path):
        self.write({'path': path})

class PensiveServer(Application):
    '''
    Tornado web application for database access over HTTP.
    '''
    def __init__(self, port, address=''):
        super(PensiveServer, self).__init__()
        self._port = port
        self._address = address

        # install handlers for various URLs
        self.add_handlers(r'.*', [(r'/(.*)', PensiveHandler)])

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

    def run(self):
        '''
        Start servicing the Tornado event loop.
        '''
        # bind the socket
        self.listen(self._port, self._address)
        logger.info('Pensive started on {}:{}'.format(
            self._address or '*', self._port))

        # NOTE: this blocks until the event loop finishes
        IOLoop.current().start()
