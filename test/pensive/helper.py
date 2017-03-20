'''
Helper for running database-dependent tests.
'''

from tornado.testing import AsyncHTTPTestCase

from pensive.server import PensiveServer
from pensive.client import PensiveClient

class FakeHTTPClient(object):  # pylint: disable=too-few-public-methods
    '''
    Tornado HTTP client wrapper to strip the protocol, host, and port
    from URLs so test cases work properly.
    '''

    def __init__(self, target):
        self._target = target
        self._trim_length = len(self._target.get_url(''))

    def fetch(self, path, **kwargs):
        return self._target.fetch(path[self._trim_length:], **kwargs)

class DatabaseDependentTestCase(AsyncHTTPTestCase):
    '''
    Unit test base class that sets up a database server and client just
    for the tests in this case.
    '''

    def setUp(self):
        '''
        Initialize the client.
        '''
        super(DatabaseDependentTestCase, self).setUp()
        self.client = PensiveClient(self.get_url(''), client=FakeHTTPClient(self))

    def get_app(self):
        '''
        Initialize the server.
        '''
        self.server = PensiveServer()
        return self.server
