'''
Raspberry Pi-based camera power control.
'''

import logging

import pigpio

from os import environ

from pensive.client import PensiveClient

CAMERA_POWER_GPIO_BCM_PIN = 26
CAMERA_POWER_GPIO_DEFAULT_PORT = 8888

logger = logging.getLogger(__name__)

class ConnectionError(RuntimeError):
    '''
    Exception represented a failed vacuum connection.
    '''
    pass

class CameraPower(object):
    '''
    CameraPpower controller for a Raspberry Pi.

    Reads host and pin from database or environment variables if not provided.
    Also ensures that the vacuum is turned off when the object is destroyed.
    '''

    def __init__(self, host=None, port=None, pin=None, store=None):
        self._store = store or PensiveClient().default()

        if self._store.get('/simulate/camera_power', True):
            self._rio = None
            logger.warn('camera power simulated')
        else:
            self._connect(host, port, pin)
            logger.info('camera power connected')

    def _connect(self, host, port, pin):
        self._pin = pin
        if not self._pin:
            # read pin from database
            self._pin = self._store.get('/config/rio/camera_power')
        if not self._pin:
            # fall pack to default
            self._pin = CAMERA_POWER_GPIO_BCM_PIN

        if not host:
            # read host from the database
            host = self._store.get('/config/rio/host')
        if not host:
            # fall back to environment variable
            host = environ.get('RIO_SERVER', None)
        if not host:
            # fall back to host name
            host = 'rio'

        if not port:
            port = CAMERA_POWER_GPIO_DEFAULT_PORT

        logger.debug('camera power using {}:{} pin {}'.format(host, port, self._pin))

        self._rio = pigpio.pi(host, port)
        # actually check the connection
        if not self._rio.connected:
            raise ConnectionError('failed to connect camera power')

        # configure vacuum pin
        self._rio.set_mode(self._pin, pigpio.OUTPUT)

    # def __del__(self):
    #     # turn the vacuum off
    #     self.off()

    #     # release resources
    #     if self._rio:
    #         logger.info('vacuum disconnected')
    #         self._rio.stop()

    def on(self):
        self.change(True)

    def off(self):
        self.change(False)

    def is_on(self):
        return self.query()

    def is_off(self):
        return not self.query()

    def change(self, on):
        '''
        Change the camera power state and update the database.
        '''
        value = False

        if isinstance(on, bool):
            value = on
        elif on.lower() == 'on':
            value = True

        # check if real or simulated vacuum
        if self._rio:
            logger.info('camera power turned {}'.format('on' if value else 'off'))
            self._rio.write(self._pin, on)
        else:
            logger.info('camera power turned {} (simulated)'.format('on' if value else 'off'))

        self._store.put('/camera_power/status', on)

    def query(self):
        '''
        Check the camera power state.
        '''
        # check if real or simulated vacuum
        if self._rio:
            return bool(self._rio.read(self._pin))
        else:
            return self._store.get('/vacuum/status')

if __name__ == '__main__':
    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter

    parser = ArgumentParser(description='power controller', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('-a', '--address', metavar='HOST', help='database server host')
    parser.add_argument('-s', '--store', metavar='STORE', help='database store')
    parser.add_argument('-c', '--camera', metavar='camera', help='camera power server host')
    parser.add_argument('command', metavar='COMMAND', type=float, help='power command 0 (off) to 1 (on)')

    args = parser.parse_args()

    # connect to the database
    from pensive.client import PensiveClient
    client = PensiveClient(args.address)

    # get the store
    store = client.store(args.store)

    host = None
    port = None
    if args.camera:
        (host, port) = args.camera.partition(':')
        if port:
            port = int(port)

    CameraPower(host, port).change(args.command > 0)
