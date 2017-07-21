'''
Gripper server interface.
'''

import logging

import socket
import struct

from os import environ

import scipy.interpolate

from pensive.client import PensiveClient

GRIPPER_DEFAULT_PORT = 5004
GRIPPER_DEFAULT_MINIMUM = 0
GRIPPER_DEFAULT_MAXIMUM = 6000

PINCH_CONTROL       = 1
CLOSE_CONTROL       = 2
SWIVEL_CONTROL      = 3

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

        encoder = [6, 20, 40, 60, 81, 100, 120, 140, 161, 181, 202, 221, 240, 260, 280]
        degrees = [-2, 5, 10, 16, 21, 29, 37, 45, 61, 67, 77, 80, 85, 87, 90]
        self.swivel_calibration = scipy.interpolate.interp1d(degrees, encoder, kind='linear')

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

    def _readall(self, n):
        data = ''
        while len(data) < n:
            chunk = self._socket.recv(n - len(data))
            if len(chunk) == 0:
                return None
            else:
                data += chunk

        return data

    def _send_packet(self, version, type_, fmt, *fields):
        payload = struct.pack(fmt, *fields)
        header = struct.pack('!BBB', version, type_, len(payload))

        packet = header + payload
        # check if real or simulated gripper
        if self._socket:
            logger.debug('gripper send: {} {}'.format(fmt, repr(payload)))
            self._socket.sendall(packet)

    def _recv_packet(self, fmt=None):
        header = self._readall(3)
        (version, type_, length) = struct.unpack('!BBB', header)

        payload = self._readall(length)
        if fmt:
            logger.debug('gripper recv: {} {}'.format(fmt, repr(payload)))
            return struct.unpack(fmt, payload)
        else:
            return payload

    def command(self, pinch=None, swivel=None, close=None):
        '''
        Change the gripper state and update the database.
        '''

        if pinch is not None:
            if pinch > 1 or pinch < 0:
                raise RuntimeError('pinch is out of range [0, 1]: {}'.format(pinch))

            q = int(round(pinch * (self._max - self._min) + self._min))
            logger.debug('gripper pinch: {}'.format(q))

            # check if real or simulated gripper
            if self._socket:
                self._send_packet(1, PINCH_CONTROL   , '!l', q)
                if self._recv_packet('!b')[0] != 1:
                    raise RuntimeError('gripper pinch failed')

            self._store.put('/gripper/pinch', pinch)

        if close is not None:
            if close > 3 or close < 1:
                raise RuntimeError('close is out of range [1, 3]: {}'.format(close))

            q = int(round(close))
            logger.debug('gripper close: {}'.format(q))

            # check if real or simulated gripper
            if self._socket:
                self._send_packet(1, CLOSE_CONTROL , '!b', q)
                if self._recv_packet('!b')[0] != 1:
                    raise RuntimeError('gripper close failed')

            self._store.put('/gripper/close', swivel)

        if swivel is not None:
            if swivel > 95 or swivel < 0:
                raise RuntimeError('swivel is out of range [0, 95]: {}'.format(swivel))

            q = int(round(self.swivel_calibration(swivel)))
            logger.debug('gripper swivel: {}'.format(q))

            # check if real or simulated gripper
            if self._socket:
                self._send_packet(1, SWIVEL_CONTROL, '!i', q)
                if self._recv_packet('!b')[0] != 1:
                    raise RuntimeError('gripper swivel failed')

            self._store.put('/gripper/swivel', swivel)

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='gripper controller', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-g', '--gripper', metavar='GRIPPER', help='gripper server host[:port]')
    parser.add_argument('command', metavar='COMMAND', choices=['pinch', 'swivel', 'close'], help='gripper command')
    parser.add_argument('value', metavar='VALUE', type=float, help='command value')

    args = parser.parse_args()

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    host = None
    port = None
    if args.gripper:
        parts = args.gripper.split(':')
        if len(parts) == 1:
            host = parts[0]
            port = None
        elif len(parts) == 2:
            host = parts[0]
            port = int(parts[1])
        else:
            raise RuntimeError('unrecognized host:port string: {}'.format(args.gripper))

    try:
        Gripper(host, port).command(**{args.command: args.value})
    except:
        logger.exception('gripper command failed')
        raise SystemExit(-1)
