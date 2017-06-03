'''
Gripper server interface.
'''

import logging

import socket
import struct

from os import environ

from pensive.client import PensiveClient

GRIPPER_DEFAULT_PORT = 5004
GRIPPER_DEFAULT_MINIMUM = 0
GRIPPER_DEFAULT_MAXIMUM = 8000

logger = logging.getLogger(__name__)

class ConnectionError(RuntimeError):
    '''
    Exception represented a failed gripper connection.
    '''
    pass

class Gripper(object):
    '''
    Gripper server client.

    Reads host and port from database or environment variables if not provided.
    '''

    def __init__(self, host=None, port=None, store=None):
        self._store = store or PensiveClient().default()

        if self._store.get('/simulate/gripper', True):
            self._socket = None
            logger.warn('gripper simulated')
        else:
            self._connect(host, port)
            logger.info('gripper connected')

        self._min = self._store.get('/config/gripper/minimum', GRIPPER_DEFAULT_MINIMUM)
        self._max = self._store.get('/config/gripper/maximum', GRIPPER_DEFAULT_MAXIMUM)

    def _connect(self, host, port):
        if not host:
            # read host from the database
            host = self._store.get('/config/gripper/host')
        if not host:
            # fall back to environment variable
            host = environ.get('GRIPPER_SERVER', None)
        if not host:
            # fall back to host name
            host = 'gripper'

        if not port:
            port = GRIPPER_DEFAULT_PORT

        logger.debug('gripper using {}:{}'.format(host, port))

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._socket.connect((host, port))
        except socket.error:
            raise ConnectionError('failed to connect gripper')

    # def __del__(self):
    #     # release resources
    #     if self._socket:
    #         logger.info('gripper disconnected')
    #         self._socket.close()

    def command(self, cmd):
        '''
        Change the gripper state and update the database.
        '''

        if cmd > 1 or cmd < 0:
            raise RuntimeError('command is out of range [0, 1]: {}'.format(cmd))

        q = int(round(cmd * (self._max - self._min) + self._min))

        # check if real or simulated gripper
        if self._socket:
            self._socket.sendall(struct.pack('!BBBl', 1, 1, 4, q))

        self._store.put('/gripper/status', cmd)

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='gripper controller', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-g', '--gripper', metavar='GRIPPER', help='gripper server host')
    parser.add_argument('command', metavar='COMMAND', type=float, help='gripper command 0 (open) to 1 (closed)')

    args = parser.parse_args()

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    host = None
    port = None
    if args.gripper:
        (host, port) = args.gripper.partition(':')
        if port:
            port = int(port)

    Gripper(host, port).command(args.command)
